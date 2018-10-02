#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"
#include "std_msgs/String.h"

#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/NestedBinVentPlanner.h"

#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeDataEntry.h"
#include "data_server/GetPlumeData.h"

#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"

#include "vent_planner/CreatePathUtil.h"

NestedBinVentPlanner::NestedBinVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    lastPlan(ros::Time::now()),
    vehicleName(vehicleName),
    dataClient(nh.serviceClient<data_server::GetData>("data_server/get")),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("data_server/get_latest")),
    plumeClient(nh.serviceClient<data_server::GetPlumeData>("data_server/get_plume")),
    goalPub(nh.advertise<std_msgs::String>("planner/goal", 1, true)),
    logPub(nh.advertise<std_msgs::String>("planner_log/log", 1000)),
    goalState("running"),
    finalSurvey(nullptr),
    phase(SearchPhase::none),
    spiralData(nullptr, 0, tf::Vector3(0,0,0), 300000, 0),
    dynamicLawnmowerController(nh, vehicleName)
{
    ROS_INFO("Waiting for data server...");
    dataClient.waitForExistence();
    latestDataClient.waitForExistence();
    plumeClient.waitForExistence();

    if(!nh.hasParam("planner/inital_spacing"))
    {
        ROS_FATAL("Parameter \"planner/inital_spacing\" not present in the parameter server.");
        exit(1);
    }

    if(!nh.hasParam("planner/final_spacing"))
    {
        ROS_FATAL("Parameter \"planner/final_spacing\" not present in the parameter server.");
        exit(1);
    }

    nh.getParam("planner/spiral_spacing", spiralSpacing);
    nh.getParam("planner/inital_spacing", initalSpacing);
    nh.getParam("planner/final_spacing", finalSpacing);
    nh.getParam("planner/fail_time", failTime);

    dataSub = nh.subscribe("data_server/" + vehicleName + "/plume_data", 0, &NestedBinVentPlanner::receivePlumeData, this);
}

void NestedBinVentPlanner::receivePlumeData(const data_server::PlumeData::ConstPtr& msg)
{
    if((phase == SearchPhase::dynamic || phase == SearchPhase::nested) &&
       dataTree)
    {
        PlumeDataEntry newPlumeData(msg->time,
                                    msg->x,
                                    msg->y,
                                    msg->h,
                                    msg->plume_strength);
        dataTree->addData(newPlumeData);
    }
    else if(phase == SearchPhase::spiral)
    {
        PlumeDataEntry newPlumeData(msg->time,
                                    msg->x,
                                    msg->y,
                                    msg->h,
                                    msg->plume_strength);
        spiralData.addData(newPlumeData);
    }


}

std::shared_ptr<Plan> NestedBinVentPlanner::plan()
{
    ROS_INFO("Plan");
    //Amount to reduce the bin size each nested pattern
    double nestedSizeFactor = 3;
    double detectionThreshold = 0.5;

    std::shared_ptr<Plan> returnPlan;

    if(phase == SearchPhase::none)
    {
        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());
            spiralPlan = returnPlan;

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
                                                                                     true);
            returnPlan->addAction(newAction);
            publishLog(vehicleName + ",Spiral");

            phase = SearchPhase::spiral;
        }
    }
    else if(phase == SearchPhase::spiral)
    {
        ROS_INFO("Update spiral plan");
        bool valid = newSpiralPlumeIntersect(spiralData, detectionThreshold);

        if(valid)
        {
            PlumeDataEntry maxVal = spiralData.getMaxVal();
            tf::Vector3 maxLoc(maxVal.x, maxVal.y, maxVal.h);
            double plumeHeight = spiralData.getHeightOfPlume();

            initalizeDataTree(maxLoc);

            returnPlan = std::shared_ptr<Plan>(new Plan());
            dynamicPlan = returnPlan;
            phase = SearchPhase::dynamic;

            const tf::Vector3 closestOrigin = dataTree->getClosestNodeOrigin(maxLoc, 1);
            addInitalLawnmowers(returnPlan, closestOrigin, plumeHeight);

            spiralData.clear();
            //Log data to file
            std::stringstream ss;
            ss.precision(5);
            ss << std::fixed << vehicleName << ",DynamicLawnmower," << plumeHeight << "," << closestOrigin.getX() << "," << closestOrigin.getY() << "," << initalSpacing;
            publishLog(ss.str());
        }
        else
        {
            returnPlan = spiralPlan;
            returnPlan->resetInterrupted();
            phase = SearchPhase::spiral;
        }
    }
    else if(phase == SearchPhase::dynamic || phase == SearchPhase::nested)
    {
        std::set<DataNode*, DataNode::PointerCompare> queuedMaxima = getUnexploredMaxima();

        if(queuedMaxima.size() > 0)
        {
            ROS_INFO("Do next maxima");
            auto lastElement = queuedMaxima.end();
            --lastElement;
            DataNode* maximum = *lastElement;

            double nestedBinSize = maximum->getSize() / nestedSizeFactor;

            ROS_INFO("Starting new maxima search; Nested Bins Size: %f, Max: %f", nestedBinSize, maximum->getMaxVal().val);
            std::vector<DataNode*> neighbors = maximum->getInitalizedNeighbors();

            //Check for goal completion.
            //This should be moved to a seperate function at some point
            ROS_INFO("Check for goal state nestedBinSize: %f, finalSpacing: %f", nestedBinSize, finalSpacing);

            bool isFinalSurvey = isGoalSurvey(nestedBinSize, maximum, neighbors);

            if(!maximum->isPartitioned())
            {
                maximum->partition(nestedSizeFactor);
            }

            for(auto neighbor : neighbors)
            {
                if(!neighbor->isPartitioned())
                {
                    neighbor->partition(nestedSizeFactor);
                }
            }

            tf::Vector3 startLocation(maximum->getCenterLocation().getX() - (maximum->getSize() * 1.5) + (nestedBinSize / 2),
                                      maximum->getCenterLocation().getY() - (maximum->getSize() * 1.5) + (nestedBinSize / 2),
                                      maximum->getHeightOfPlume());

            std::vector<tf::Vector3> nestedPattern = create_path_util::makeLawnmower(startLocation,
                                                                   0,
                                                                   M_PI / 2,
                                                                   (nestedSizeFactor * 3 - 1) * nestedBinSize,
                                                                   (nestedSizeFactor * 3 - 1) * nestedBinSize,
                                                                   nestedBinSize);

            std::shared_ptr<PointPathAction> lawnmowerAction = actionFactory->createPointPathAction(vehicleName,
                                                                                                    1.0,
                                                                                                    0.349066,
                                                                                                    0.523599, //30 deg
                                                                                                    nestedPattern,
                                                                                                    false);

            ROS_INFO("New nested lawnmower, Current Point: %i, Total Points: %lu", lawnmowerAction->getCurrentPoint(), nestedPattern.size());
            returnPlan = std::shared_ptr<Plan>(new Plan());
            returnPlan->addAction(lawnmowerAction);
            plannedMaxima.insert(std::make_pair(returnPlan, maximum));
            phase = SearchPhase::nested;

            if(isFinalSurvey)
            {
                finalSurvey = returnPlan;
            }

            std::stringstream ss;
            ss.precision(5);
            ss << std::fixed << vehicleName << ",NestedLawnmower," << startLocation.getZ() << "," << startLocation.getX() << "," << startLocation.getY() << "," << nestedBinSize;
            publishLog(ss.str());
        }
        else
        {
            if(!dynamicPlan->isCompleted())
            {
                returnPlan = dynamicPlan;
                phase = SearchPhase::dynamic;
            }
            else
            {
                returnPlan = spiralPlan;
                phase = SearchPhase::spiral;
                ROS_INFO("Resume Spiral");
            }
        }
    }

    if(finalSurvey && finalSurvey->isCompleted())
    {
        goalState = "success";
        ROS_INFO("Set goal state: success");
    }

    //updates the goal state and publishes it
    updateGoal();
    publishGoal();

    if(returnPlan)
    {
        lastPlan = ros::Time::now();
    }

    return returnPlan;
}

bool NestedBinVentPlanner::newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold)
{
    PlumeDataEntry maxVal = spiralData.getMaxVal();
    bool intersect = false;
    if(maxVal.val >= detectionThreshold)
    {
        if(!dataTree)
        {
            intersect = true;
        }
        else
        {
            //Set valid to false if this area has already been investigated
            DataNode& smallestAtMaxVal = dataTree->getSmallestNode(tf::Vector3(maxVal.x, maxVal.y, maxVal.h));
            if(smallestAtMaxVal.getNodeLevel() < 1)
            {
                intersect = true;
            }
        }
    }

    return intersect;
}

void NestedBinVentPlanner::initalizeDataTree(tf::Vector3 centerLocation)
{
    if(!dataTree)
    {
        float targetSize = 300000;
        int numPartitions = ceil(targetSize / initalSpacing);

        dataTree = std::unique_ptr<DataTree>(new DataTree(centerLocation,
                                                          numPartitions * initalSpacing));
        dataTree->getRoot().partition(numPartitions);
        ROS_INFO("Initalize data bins, numPartitions: %i, size: %f", numPartitions, numPartitions * initalSpacing);
    }
}

bool NestedBinVentPlanner::getLatestData(DataServerEntry& returnEntry)
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

std::set<DataNode*, DataNode::PointerCompare> NestedBinVentPlanner::getUnexploredMaxima()
{
    std::set<DataNode*, DataNode::PointerCompare> queuedMaxima;
    const std::vector<DataNode*> binMaxima = dataTree->getMaxima();
    ROS_INFO("Maxima Found: %lu", binMaxima.size());
    for(unsigned int i = 0; i < binMaxima.size(); i++)
    {
        bool found = false;
        for (auto it = plannedMaxima.begin(); it != plannedMaxima.end(); ++it)
        {
            if(*(it->second) == *binMaxima[i])
            {
                ROS_INFO("Check plannedMaxima Found: %p, Val: %f, X: %f, Y: %f, Level: %i",(void*)binMaxima[i], binMaxima[i]->getMaxVal().val, binMaxima[i]->getCenterLocation().getX(),
                         binMaxima[i]->getCenterLocation().getY(),
                         binMaxima[i]->getNodeLevel());
                found = true;
                break;
            }
        }

        if(!found && binMaxima[i]->getSize() > finalSpacing)
        {
            unsigned long queueSize = queuedMaxima.size();
            queuedMaxima.insert(binMaxima[i]);

            ROS_INFO("Added maximum to queue: %p, Val: %f, X: %f, Y: %f, Level: %i", (void*)binMaxima[i], binMaxima[i]->getMaxVal().val,
                     binMaxima[i]->getCenterLocation().getX(),
                     binMaxima[i]->getCenterLocation().getY(),
                     binMaxima[i]->getNodeLevel());
        }
    }

    return queuedMaxima;
}

void NestedBinVentPlanner::addInitalLawnmowers(std::shared_ptr<Plan> plan, const tf::Vector3& centerLocation, double plumeHeight)
{
    tf::Vector3 lawnmower0Start(centerLocation.getX() + initalSpacing / 2.0, centerLocation.getY() + initalSpacing / 2.0, plumeHeight);
    tf::Vector3 lawnmower1Start(centerLocation.getX() - initalSpacing / 2.0, centerLocation.getY() + initalSpacing / 2.0, plumeHeight);
    tf::Vector3 lawnmower2Start(centerLocation.getX() - initalSpacing / 2.0, centerLocation.getY() - initalSpacing / 2.0, plumeHeight);
    tf::Vector3 lawnmower3Start(centerLocation.getX() + initalSpacing / 2.0, centerLocation.getY() - initalSpacing / 2.0, plumeHeight);

    std::shared_ptr<Action> lawnmower0 = actionFactory->createDynamicLawnmowerAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower0Start,
                                                                                     0,
                                                                                     M_PI / 2,
                                                                                     initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower1 = actionFactory->createDynamicLawnmowerAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower1Start,
                                                                                     M_PI,
                                                                                     M_PI / 2,
                                                                                     initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower2 = actionFactory->createDynamicLawnmowerAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower2Start,
                                                                                     M_PI,
                                                                                     M_PI * 3 / 2,
                                                                                     initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower3 = actionFactory->createDynamicLawnmowerAction(vehicleName,
                                                                                     1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower3Start,
                                                                                     0,
                                                                                     M_PI * 3 / 2,
                                                                                     initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);
                                                                                     

    plan->addAction(lawnmower0);
    plan->addAction(lawnmower1);
    plan->addAction(lawnmower2);
    plan->addAction(lawnmower3);
}

bool NestedBinVentPlanner::isGoalSurvey(double nestedBinSize, DataNode* maximum, std::vector<DataNode*>& neighbors)
{
    bool goalSurvey = false;
    if(nestedBinSize <= finalSpacing + 0.1)
    {
        DataNode& smallestCenter = dataTree->getSmallestNode(tf::Vector3(0,0,0));

        if(maximum == &smallestCenter)
        {
            goalSurvey = true;
        }

        for(auto neighbor : neighbors)
        {
            if(&smallestCenter == neighbor)
            {
                ROS_INFO("Set final survey");
                goalSurvey = true;
            }
        }
    }

    return goalSurvey;
}

void NestedBinVentPlanner::publishGoal()
{
    std_msgs::String msg;
    msg.data = goalState;
    goalPub.publish(msg);
}

void NestedBinVentPlanner::updateGoal()
{
    if(goalState == "running")
    {
        if(ros::Time::now() >= ros::Time(failTime))
        {
            goalState = "failed";
        }
    }
}

void NestedBinVentPlanner::publishLog(std::string log)
{
    std_msgs::String msg;

    std::stringstream ss;
    ss.precision(5);
    ss << std::fixed << ros::Time::now().toSec() << ": " << log;
    msg.data =  ss.str();
    logPub.publish(msg);
}