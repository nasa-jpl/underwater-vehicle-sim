#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"
#include "std_msgs/String.h"

#include "vent_planner/DataNode.h"

#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"
#include "data_server/DataServerEntry.h"
#include "data_server/GetPlumeData.h"
#include "data_server/PlumeData.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"

#include "vent_planner/util/CreatePathUtil.h"
#include "vent_planner/util/MathUtil.h"
#include "vent_planner/util/Plane.h"

SurfaceGradientVentPlanner::SurfaceGradientVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    vehicleName(vehicleName),
    goalState("running"),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("data_server/get_latest")),
    goalPub(nh.advertise<std_msgs::String>("planner/goal", 1, true)),
    currentPlannerStage(SearchPhase::INITIAL_PLAN),
    spiralData(nullptr, 0, tf::Vector3(0,0,0), 300000, 0)
{
    nh.getParam("planner/fail_time", failTime);
    nh.getParam("planner/spiral_spacing", spiralSpacing);

    nh.getParam("planner/detection_threshold", detectionThreshold);
    nh.getParam("planner/gradient_radius", gradientCalcRadius);
    nh.getParam("planner/gradient_threshold", gradientThreshold);
    nh.getParam("planner/max_follow_distance", gradientMaxFollowDistance);
    nh.getParam("planner/min_follow_distance", gradientMinFollowDistance);
    nh.getParam("planner/gradient_window", gradientWindow);

    dataSub = nh.subscribe("data_server/" + vehicleName + "/plume_data", 0, &SurfaceGradientVentPlanner::receivePlumeData, this);
}

void SurfaceGradientVentPlanner::receivePlumeData(const data_server::PlumeData::ConstPtr& msg)
{
    if(currentPlannerStage == SearchPhase::SPIRAL)
    {
        PlumeDataEntry newPlumeData(msg->time,
                                    msg->x,
                                    msg->y,
                                    msg->h,
                                    msg->plume_strength);
        spiralData.addData(newPlumeData);
    }
    else
    {
        if(!std::isnan(msg->x) &&
           !std::isnan(msg->y) &&
           !std::isnan(msg->plume_strength))
        {
            currentData.emplace_back(msg->x,
                                 msg->y,
                                 msg->plume_strength); 
        }
    }
}

bool SurfaceGradientVentPlanner::endGradientFollow()
{
    if(currentData.size() == 0)
    {
        return false;
    }

    tf::Vector3& lastPoint = currentData[currentData.size() - 1];
    
    //Check that we are at least a window size past the initial point on the circle's edge
    if(gradientFollowAction->getCurrentPoint() >= 1 &&
        math_util::xyDistance(lastPoint, gradientFollowAction->points[0]) >= gradientWindow &&
        math_util::xyDistance(lastPoint, gradientFollowAction->points[0]) >= gradientMinFollowDistance)
    {
        
        //Find start of data to use for follow gradient calculation
        unsigned int start = currentData.size() - 1;
        while(start >= 0)
        {
            if(!std::isnan(currentData[start].getX()) &&
               !std::isnan(currentData[start].getY()))
            {
                if(math_util::xyDistance(currentData[start], lastPoint) >= gradientWindow)
                {
                    break;
                }
            }
            start--;
        }

        if(start < 0)
        {
            start = 0;
        }

        std::vector<double> x;
        std::vector<double> y;
        double slope = 0.0;
        double yIntercept = 0.0;

        //Caluclate x and y values for follow graidnet calculation
        for(unsigned int i = start; i < currentData.size(); i++)
        {
            if(!std::isnan(currentData[i].getX()) &&
               !std::isnan(currentData[i].getY()))
            {
                x.push_back(math_util::xyDistance(currentData[start], currentData[i]));
                y.push_back(currentData[i].getZ());
            }
        }
        
        math_util::linearLeastSquares(x, y, slope, yIntercept);

        ROS_INFO("Last point distance: %f, window: %f, min: %f", math_util::xyDistance(lastPoint, gradientFollowAction->points[0]), gradientWindow, gradientMinFollowDistance);
        ROS_INFO("Last Point x: %f, y: %f", lastPoint.getX(), lastPoint.getY());
        ROS_INFO("Action Point x: %f, y: %f, z: %f", gradientFollowAction->points[0].getX(), gradientFollowAction->points[0].getY(), gradientFollowAction->points[0].getZ());
        ROS_INFO("Follow Phase calculated gradient: %f, threshold: %f", slope, gradientThreshold);

        return slope < gradientThreshold;
    }

    return false;
}

std::shared_ptr<Plan> SurfaceGradientVentPlanner::plan()
{
    ROS_INFO("Plan");

    std::shared_ptr<Plan> returnPlan(nullptr);


    if(currentPlannerStage == SearchPhase::INITIAL_PLAN)
    {
        ROS_INFO("Phase INITIAL_PLAN");
        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());

            ROS_INFO("Generate inital plan");
            tf::Vector3 vehicleLocation(latestEntry.x, latestEntry.y, latestEntry.h);
            std::vector<tf::Vector3> spiralPoints = create_path_util::makeSpiral(vehicleLocation, 0, spiralSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     -100,
                                                                                     -2000,
                                                                                     spiralPoints,
                                                                                     PointPathAction::ReplanType::ON_YOYO_TURN,
                                                                                     0);
            returnPlan->addAction(newAction);

            currentPlannerStage = SearchPhase::SPIRAL;
        }
        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::SPIRAL)
    {
        ROS_INFO("Phase SPIRAL");
        PlumeDataEntry maxVal = spiralData.getMaxVal();
        if(maxVal.val >= detectionThreshold)
        {
            tf::Vector3 maxLoc(maxVal.x, maxVal.y, maxVal.h);
            plumeHeight = spiralData.getHeightOfPlume();

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::PLAN_GRADIENT;     
        }

        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::PLAN_GRADIENT)
    {
        ROS_INFO("Phase PLAN_GRADIENT");
        currentData.clear();
        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());
            gradientCirclePlan = returnPlan;

            ROS_INFO("Generate gradient plan");
            gradientLocation.setX(latestEntry.x);
            gradientLocation.setY(latestEntry.y);
            gradientLocation.setZ(plumeHeight);

            std::vector<tf::Vector3> gradientPoints = create_path_util::makePolygon(gradientLocation,
                                                   8, //sides
                                                   gradientCalcRadius,
                                                   0, //initial heading
                                                   true); //clockwise

            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     gradientPoints,
                                                                                     PointPathAction::ReplanType::NONE,
                                                                                     0);
            returnPlan->addAction(newAction);

            currentPlannerStage = SearchPhase::CALC_GRADIENT;
        }
    }
    else if(currentPlannerStage == SearchPhase::CALC_GRADIENT)
    {
        ROS_INFO("Phase CALC_GRADIENT");
        if(gradientCirclePlan->isCompleted())
        {
            Plane fitPlane = Plane::fitPlaneToPoints(currentData);
            gradientDirection = fitPlane.getHeightGradientHeading();

            //flip direction as gradient points downhill, but
            //we want to go up
            gradientDirection += M_PI; 

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::FOLLOW_GRADIENT;
        }
    }
    else if(currentPlannerStage == SearchPhase::FOLLOW_GRADIENT)
    {
        ROS_INFO("Phase FOLLOW_GRADIENT");
        returnPlan = std::shared_ptr<Plan>(new Plan());

        //Save the plan so we can check if it is complete later
        gradientFollowPlan = returnPlan;

        tf::Vector3 vecHeading(sin(gradientDirection), cos(gradientDirection), 0);
        vecHeading.normalize(); //Do we need this??
        tf::Vector3 p1 = gradientLocation + vecHeading * gradientCalcRadius;
        tf::Vector3 p2 = p1 + vecHeading * gradientMaxFollowDistance;

        ROS_INFO("HEIGHT: %f %f %f", p1.getZ(), p2.getZ(), plumeHeight);
        std::vector<tf::Vector3> linePoints;
        linePoints.push_back(p1);
        linePoints.push_back(p2);


       
        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                          1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          linePoints,
                                                                                          PointPathAction::ReplanType::PERIODIC,
                                                                                          200);

         //Save this action in a shared ptr so we can check how much we have completed later
        gradientFollowAction = newAction;

        returnPlan->addAction(newAction);

        //Move to observing the single dimensional gradient
        currentPlannerStage = SearchPhase::OBSERVE_FOLLOW_GRADIENT;
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_FOLLOW_GRADIENT)
    {
        ROS_INFO("Phase OBSERVE_FOLLOW_GRADIENT");
        if(endGradientFollow() ||
           gradientFollowPlan->isCompleted())
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::PLAN_GRADIENT;
        }
    }

    //updates the goal state and publishes it
    updateGoal();
    publishGoal();

    return returnPlan;
}

void SurfaceGradientVentPlanner::publishGoal()
{
    std_msgs::String msg;
    msg.data = goalState;
    goalPub.publish(msg);
}

void SurfaceGradientVentPlanner::updateGoal()
{
    if(goalState == "running")
    {
        if(ros::Time::now() >= ros::Time(failTime))
        {
            goalState = "failed";
        }
    }
}

bool SurfaceGradientVentPlanner::getLatestData(DataServerEntry& returnEntry)
{
    data_server::GetLatestData srv;
    srv.request.name = vehicleName;

    bool valid = latestDataClient.call(srv);
    if(valid)
    {
        returnEntry.x = srv.response.x;
        returnEntry.y = srv.response.y;
        returnEntry.h = srv.response.h;

        returnEntry.time = srv.response.time;
        returnEntry.temp = srv.response.temp;
        returnEntry.salt = srv.response.salt;
        returnEntry.dye = srv.response.dye;

        return true;
    }

    return false;
}