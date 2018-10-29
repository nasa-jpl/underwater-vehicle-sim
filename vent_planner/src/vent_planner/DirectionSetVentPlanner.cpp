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
#include "vent_planner/DirectionSetVentPlanner.h"

#include "vent_planner/util/CreatePathUtil.h"
#include "vent_planner/util/MathUtil.h"
#include "vent_planner/util/Plane.h"

DirectionSetVentPlanner::DirectionSetVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, VehicleInfo vehicleInfo) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    vehicleInfo(vehicleInfo),
    goalState(GoalState::RUNNING),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("data_server/get_latest")),
    goalPub(nh.advertise<std_msgs::String>("planner/goal", 1, true)),
    currentPlannerStage(SearchPhase::INITIAL_PLAN),
    spiralData(nullptr, 0, tf::Vector3(0,0,0), 300000, 0),
    pointPathController(nh, vehicleInfo)
{
    nh.getParam("planner/fail_time", failTime);
    nh.getParam("planner/spiral_spacing", spiralSpacing);
    nh.getParam("planner/detection_threshold", detectionThreshold);
    nh.getParam("planner/min_leg_length", minLegLength);
    nh.getParam("planner/max_leg_length", maxLegLength);
    nh.getParam("planner/leg_section_length", legSectionLength);
    nh.getParam("planner/new_max_threshold", newMaxThreshold);
    nh.getParam("planner/num_sections_threshold", numSectionsThreshold);

    dataSub = nh.subscribe("data_server/" + vehicleInfo.getName() + "/plume_data", 0, &DirectionSetVentPlanner::receivePlumeData, this);
}

void DirectionSetVentPlanner::receivePlumeData(const data_server::PlumeData::ConstPtr& msg)
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

            if(msg->plume_strength > currentMax.getZ())
            {
                tf::Vector3 testVector(msg->x, msg->y, 0);
                if(math_util::xyDistance(testVector, currentMax) > newMaxThreshold)
                {
                    currentMax.setX(msg->x);
                    currentMax.setY(msg->y);
                    currentMax.setZ(msg->plume_strength);
                    numMaxCrossings = 1;
                }
            } 
        }
    }
}

bool DirectionSetVentPlanner::endTransect()
{
    tf::Vector3& lastPoint = currentData[currentData.size() - 1];

    //Check to insure no violation of min and max leg lengths
    if(math_util::xyDistance(lastPoint, currentLineCenter) < minLegLength)
    {
        return false;
    }
    else if(math_util::xyDistance(lastPoint, currentLineCenter) >= maxLegLength)
    {
        return true;
    }
    

    int start = currentData.size() - 1;
    while(start >= 0 &&
          math_util::xyDistance(currentData[start], lastPoint) < legSectionLength * numSectionsThreshold)
    {
        start--;
    }
    
    ROS_INFO("End Transect: start: %f x: %f y: %f z: %f", math_util::xyDistance(currentData[start], lastPoint), currentData[start].getX(), currentData[start].getY(), currentData[start].getZ());
    //Don't end transect because we have not travelled far
    //enough to do all section calculations
    if(start == -1)
    {
        ROS_INFO("End Transect: Not far enough");
        return false;
    }

    std::vector<double> average(numSectionsThreshold);
    std::vector<unsigned int> dataCount(numSectionsThreshold);

    tf::Vector3&  startPoint = currentData[start];
    for(unsigned int i = start; i < currentData.size() - 1; i++)
    {
        unsigned int section = math_util::xyDistance(currentData[i], startPoint) / legSectionLength;
        if(section < numSectionsThreshold)
        {
            average[section] += currentData[i].getZ();
            dataCount[section]++;
        }
    }

    double lastAverage = detectionThreshold;
    for(unsigned int i = 0; i < average.size(); i++)
    {
        ROS_INFO("End Transect Before Calc: Average: %f count: %u", average[i], dataCount[i]);
        average[i] /= dataCount[i];
        ROS_INFO("End Transect: Average: %f lastAverage: %f", average[i], lastAverage);
        if(average[i] > lastAverage)
        {
            return false;
        }
        lastAverage = average[i];
    }
    
    return true;
}

std::shared_ptr<Plan> DirectionSetVentPlanner::plan()
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
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleInfo.getName(),
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
            currentMax.setX(maxVal.x);
            currentMax.setY(maxVal.y);
            currentMax.setZ(maxVal.val);
            numMaxCrossings = 0;
            currentHeading = 0;

            plumeHeight = spiralData.getHeightOfPlume();

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::EXECUTE_LINE_0;     
        }

        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_0)
    {
        //Update heading and go to center of new transect

        returnPlan = std::shared_ptr<Plan>(new Plan());

        ROS_INFO("Phase EXECUTE_LINE");
        if(numMaxCrossings <= 1)
        {
            currentHeading += M_PI / 2;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            }
        }
        else if(numMaxCrossings == 2)
        {
            //Do line at 45 degrees
            currentHeading += M_PI / 4;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            } 
        }
        else if(numMaxCrossings == 3)
        {
            //Do other 45 degree line
            currentHeading += M_PI / 2;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            } 
        }
        else
        {
            goalState = GoalState::SUCCESS;
        }
        numMaxCrossings++;

        currentLineCenter = currentMax;
        std::vector<tf::Vector3> points;
        tf::Vector3 p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        points.push_back(p1);
        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(vehicleInfo.getName(),
                                                                                  1.0,
                                                                                  0.349066,
                                                                                  0.523599, //30 deg
                                                                                  points,
                                                                                  PointPathAction::ReplanType::NONE,
                                                                                  0);

        returnPlan->addAction(newAction);
        currentPlannerStage = SearchPhase::EXECUTE_LINE_1;
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_1)
    {
        //Go to first end of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());

        std::vector<tf::Vector3> points;
        tf::Vector3 vecHeading1(sin(currentHeading), cos(currentHeading), 0);

        tf::Vector3 p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        tf::Vector3 p2 = p1 + vecHeading1 * maxLegLength;

        points.push_back(p1);
        points.push_back(p2);

        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(vehicleInfo.getName(),
                                                                                  1.0,
                                                                                  0.349066,
                                                                                  0.523599, //30 deg
                                                                                  points,
                                                                                  PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                  legSectionLength);

        returnPlan->addAction(newAction);
        transectPlan = returnPlan;
        currentData.clear();

        currentPlannerStage = SearchPhase::OBSERVE_EXECUTE_LINE_1; 
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_EXECUTE_LINE_1)
    {
        if(transectPlan->isCompleted() || endTransect())
        {
            //Send back empty plan to trigger replan again (not an ideal way to do this)
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::EXECUTE_LINE_2;
        }
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_2)
    {
        //Go to center of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());
        std::vector<tf::Vector3> points;
        tf::Vector3 p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        points.push_back(p1);
        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(vehicleInfo.getName(),
                                                                                  1.0,
                                                                                  0.349066,
                                                                                  0.523599, //30 deg
                                                                                  points,
                                                                                  PointPathAction::ReplanType::NONE,
                                                                                  0);

        returnPlan->addAction(newAction);
        currentPlannerStage = SearchPhase::EXECUTE_LINE_3;
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_3)
    {
        //Go to second end of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());

        std::vector<tf::Vector3> points;
        tf::Vector3 vecHeading2(sin(currentHeading + M_PI), cos(currentHeading + M_PI), 0);

        tf::Vector3 p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        tf::Vector3 p3 = p1 + vecHeading2 * maxLegLength;

        points.push_back(p1);
        points.push_back(p3);

        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(vehicleInfo.getName(),
                                                                                  1.0,
                                                                                  0.349066,
                                                                                  0.523599, //30 deg
                                                                                  points,
                                                                                  PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                  legSectionLength);

        returnPlan->addAction(newAction);
        transectPlan = returnPlan;
        currentData.clear();

        currentPlannerStage = SearchPhase::OBSERVE_EXECUTE_LINE_3;
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_EXECUTE_LINE_3)
    {
        if(transectPlan->isCompleted() || endTransect())
        {
            //Send back empty plan to trigger replan again  (not an ideal way to do this)
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::EXECUTE_LINE_0;
        }
    }

    //updates the goal state and publishes it
    updateGoal();
    publishGoal();

    return returnPlan;
}

void DirectionSetVentPlanner::publishGoal()
{
    std_msgs::String msg;
    if(goalState == GoalState::RUNNING)
    {
       msg.data = "running"; 
    }
    else if(goalState == GoalState::FAILED)
    {
       msg.data = "failed"; 
    }
    else if(goalState == GoalState::SUCCESS)
    {
       msg.data = "success"; 
    }
    else
    {
       msg.data = "invalid"; 
    }

    goalPub.publish(msg);
}

void DirectionSetVentPlanner::updateGoal()
{
    if(goalState == GoalState::RUNNING)
    {
        if(ros::Time::now() >= ros::Time(failTime))
        {
            goalState = GoalState::FAILED;
        }
    }
}

bool DirectionSetVentPlanner::getLatestData(DataServerEntry& returnEntry)
{
    data_server::GetLatestData srv;
    srv.request.name = vehicleInfo.getName();

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