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

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/NestedBinVentPlanner.h"

#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeData.h"
#include "plume_detector/GetPlumeData.h"

#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"

NestedBinVentPlanner::NestedBinVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    lastPlan(ros::Time::now()),
    initalPlan(false),
    vehicleName(vehicleName),
    dataClient(nh.serviceClient<data_server::GetData>("/data_server/get")),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("/data_server/get_latest")),
    plumeClient(nh.serviceClient<plume_detector::GetPlumeData>("/plume_detector/get")),
    goalPub(nh.advertise<std_msgs::String>("planner/goal", 1, true)),
    spiralData(nullptr, 0, tf::Vector3(0,0,0), 300000, 0),
    goalState("running"),
    finalSurvey(nullptr)
{
    ROS_INFO("Planner: Waiting for data server...");
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
}

std::shared_ptr<Plan> NestedBinVentPlanner::plan()
{
    //Amount to reduce the bin size each nested pattern
    double nestedSizeFactor = 3;

    ROS_INFO("Planner: Plan");

    std::shared_ptr<Plan> createdPlan;

    //Create inital plan and push it onto the stack
    if(!initalPlan && plans.size() == 0)
    {   
        createdPlan = std::shared_ptr<Plan>(new Plan());

        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            ROS_INFO("Planner: Generate inital plan");
            tf::Vector3 vehicleLocation(latestEntry.x, latestEntry.y, latestEntry.h);
            std::vector<tf::Vector3> spiralPoints = makeSpiral(vehicleLocation, 0, spiralSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                    1.0,
                                                                                    0.349066,
                                                                                    0.523599, //30 deg
                                                                                    -100,
                                                                                    -2000,
                                                                                    spiralPoints,
                                                                                    true);
            createdPlan->addAction(newAction);
            initalPlan = true;
        }
    }
    else
    {
        plume_detector::GetPlumeData srv;
        srv.request.name = vehicleName;
        srv.request.start_time = lastPlan;
        srv.request.end_time = ros::Time::now();

        plumeClient.call(srv);

        if(plans.size() == 1) //If on spiral, save data to the spiralData bin 
        {
            spiralData.clear(); //Clear data so we can easily calculate plume height and max value
            for(unsigned int i = 0; i < srv.response.time.size(); i++)
            {
               PlumeData newPlumeData(srv.response.time[i],
                              srv.response.x[i],
                              srv.response.y[i],
                              srv.response.h[i],
                              srv.response.val[i]);
               spiralData.addData(newPlumeData);
            }

            PlumeData maxVal = spiralData.getMaxVal();

            bool valid = maxVal.val >= 0.5;
            //Set valid to false if this area has already been investigated
            if(valid && dataTree)
            {
                DataNode& smallestAtMaxVal = dataTree->getSmallestNode(tf::Vector3(maxVal.x, maxVal.y, maxVal.h));
                if(smallestAtMaxVal.getNodeLevel() >= 1)
                {
                    valid = false;
                }
            }

            if(valid)
            {
                tf::Vector3 maxLoc(maxVal.x, maxVal.y, maxVal.h);
                double plumeHeight = spiralData.getHeightOfPlume();

                //Create bins
                if(!dataTree)
                {
                    ROS_INFO("Planner: Initalize data bins");
                    dataTree = std::unique_ptr<DataTree>(new DataTree(maxLoc,
                                                                      300000));
                    dataTree->getRoot().partition(300000 / initalSpacing);
                }

                ROS_INFO("Planner: Start lawnmowers");

                createdPlan = std::shared_ptr<Plan>(new Plan());

                addInitalLawnmowers(createdPlan, maxLoc, plumeHeight);           
            }
        }
        else if(plans.size() > 1) //Add data to dataBins if we are no longer on the first spiral
        {
            if(dataTree)
            {
                for(unsigned int i = 0; i < srv.response.time.size(); i++)
                {
                   PlumeData newPlumeData(srv.response.time[i],
                                  srv.response.x[i],
                                  srv.response.y[i],
                                  srv.response.h[i],
                                  srv.response.val[i]);
                   dataTree->addData(newPlumeData);
                }
            }


            std::set<DataNode*, DataNode::PointerCompare> queuedMaxima;
            const std::vector<DataNode*> binMaxima = dataTree->getMaxima();
            ROS_INFO("Planner: Maxima Found: %lu", binMaxima.size());
            for(unsigned int i = 0; i < binMaxima.size(); i++)
            {
                bool found = false;
                for (auto it = plannedMaxima.begin(); it != plannedMaxima.end(); ++it)
                {
                    if(*(it->second) == *binMaxima[i])
                    {
                        ROS_INFO("Planner: Check plannedMaxima Found: %p, Val: %f, X: %f, Y: %f, Level: %i",(void*)binMaxima[i], binMaxima[i]->getMaxVal().val, binMaxima[i]->getCenterLocation().getX(),
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

                    ROS_INFO("Planner: Added maximum to queue: %p, Val: %f, X: %f, Y: %f, Level: %i", (void*)binMaxima[i], binMaxima[i]->getMaxVal().val, 
                                                                                                                                   binMaxima[i]->getCenterLocation().getX(),
                                                                                                                                   binMaxima[i]->getCenterLocation().getY(),
                                                                                                                                   binMaxima[i]->getNodeLevel());
                }
            }
            
            //if there is no current maximum being investigated or the current maximum is less than the new maximum
            if(queuedMaxima.size() > 0)
            {
                ROS_INFO("Planner: Do next maxima");
                auto lastElement = queuedMaxima.end();
                --lastElement;
                DataNode* maximum = *lastElement;
                queuedMaxima.erase(lastElement);

                double nestedBinSize = maximum->getSize() / nestedSizeFactor;

                ROS_INFO("Planner: Starting new maxima search; Nested Bins Size: %f, Max: %f", nestedBinSize, maximum->getMaxVal().val);
                std::vector<DataNode*> neighbors = maximum->getInitalizedNeighbors();

                //Check for goal completion.
                //This should be moved to a seperate function at some point
                ROS_INFO("Planner: Check for goal state nestedBinSize: %f, finalSpacing: %f", nestedBinSize, finalSpacing);
                bool isFinalSurvey = false;
                if(nestedBinSize <= finalSpacing + 0.1)
                {
                    DataNode& smallestCenter = dataTree->getSmallestNode(tf::Vector3(0,0,0));
                    ROS_INFO("Planner: Smallest Node: X: %f Y: %f Size: %f", smallestCenter.getCenterLocation().getX(),
                                                                             smallestCenter.getCenterLocation().getY(),
                                                                             smallestCenter.getSize());
                    

                    ROS_INFO("Planner: Check for goal state smallestCenter: %p, maximum: %p", (void*)(&smallestCenter), (void*)maximum);
                    ROS_INFO("Planner: Check for goal state smallestCenter: %f %f %f, maximum: %f %f %f", smallestCenter.getCenterLocation().getX(), 
                                                                                                         smallestCenter.getCenterLocation().getY(), 
                                                                                                         smallestCenter.getSize(),
                                                                                                         maximum->getCenterLocation().getX(),
                                                                                                         maximum->getCenterLocation().getY(),
                                                                                                         maximum->getSize());
                    if(maximum == &smallestCenter)
                    {
                        isFinalSurvey = true;
                    }

                    for(auto neighbor : neighbors)
                    {
                        ROS_INFO("Planner: Check for goal state smallestCenter: %p, neighbor: %p", (void*)(&smallestCenter), (void*)neighbor);
                        ROS_INFO("Planner: Check for goal state smallestCenter: %f %f %f, neighbor: %f %f %f", smallestCenter.getCenterLocation().getX(), 
                                                                                                         smallestCenter.getCenterLocation().getY(), 
                                                                                                         smallestCenter.getSize(),
                                                                                                         neighbor->getCenterLocation().getX(),
                                                                                                         neighbor->getCenterLocation().getY(),
                                                                                                         neighbor->getSize());
                        if(&smallestCenter == neighbor)
                        {
                            ROS_INFO("Planner: Set final survey");
                            isFinalSurvey = true;
                        }
                    }
                }                

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

                std::vector<tf::Vector3> nestedPattern = makeLawnmower(startLocation,
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

                ROS_INFO("Planner: New nested lawnmower, Current Point: %i, Total Points: %lu", lawnmowerAction->getCurrentPoint(), nestedPattern.size());
                createdPlan = std::shared_ptr<Plan>(new Plan());

                createdPlan->addAction(lawnmowerAction);
                plannedMaxima.insert(std::make_pair(createdPlan, maximum));
                if(isFinalSurvey)
                {
                    finalSurvey = createdPlan;
                }
                
            }
        }
    }

    

    //Pop the top plan if it has been completed
    while(plans.size() > 0 && isCompleted(plans.top()))
    {
        ROS_INFO("Planner: Finished plan");

        if(finalSurvey && finalSurvey == plans.top())
        {
            goalState = "success";
            ROS_INFO("Planner: Set goal state: success");
        }
        plans.pop();
    }

    //updates the goal state and publishes it
    updateGoal();
    publishGoal();

    lastPlan = ros::Time::now();

    if(createdPlan != nullptr)
    {
        plans.push(createdPlan);
        return createdPlan;
    }
    else if(plans.size() > 0)
    {
        plans.top()->resetInterrupted();
        ROS_INFO("Planner: Top plan sent");
        return plans.top();
    }

     ROS_INFO("Planner: NUll plan sent");
    return nullptr;
}

void NestedBinVentPlanner::addInitalLawnmowers(std::shared_ptr<Plan> plan, tf::Vector3& centerLocation, double plumeHeight)
{
    tf::Vector3 lawnmower0Start(centerLocation.getX() + initalSpacing / 2, centerLocation.getY() + initalSpacing / 2, plumeHeight);
    tf::Vector3 lawnmower1Start(centerLocation.getX() - initalSpacing / 2, centerLocation.getY() + initalSpacing / 2, plumeHeight);
    tf::Vector3 lawnmower2Start(centerLocation.getX() - initalSpacing / 2, centerLocation.getY() - initalSpacing / 2, plumeHeight);
    tf::Vector3 lawnmower3Start(centerLocation.getX() + initalSpacing / 2, centerLocation.getY() - initalSpacing / 2, plumeHeight);

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

bool NestedBinVentPlanner::isCompleted(std::shared_ptr<Plan> plan)
{
    ROS_INFO("Planner: Check for completed");
    for(auto action : plan->getActions())
    {
        if(!(action->getState() == Action::State::COMPLETED || 
             action->getState() == Action::State::FAILED))
        {
            ROS_INFO("Planner: Not completed");
            return false;
        }
    }
    ROS_INFO("Planner: Completed");
    return true;
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

std::vector<tf::Vector3> NestedBinVentPlanner::makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size)
{
    std::vector<tf::Vector3> spiral;
    spiral.push_back(startLocation);

    tf::Vector3 location = startLocation;

    const std::vector<double> directions = {startDirection, 
                                            startDirection + (M_PI / 2), 
                                            startDirection + M_PI, 
                                            startDirection + (M_PI * 3 / 2)};

    unsigned int currentDirection = 0;
    unsigned int lengthIndex = 1;

    while(lengthIndex * spacing <= size)
    {
        //Transect1 at transectLength
        location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
        location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
        location.setZ(startLocation.getZ());
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        //Transect2 at transectLength
        location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
        location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
        location.setZ(startLocation.getZ());
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        lengthIndex++;
    }

    //Final transect to finish out the spiral, same transect length as the last segment
    location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    location.setZ(startLocation.getZ());
    spiral.push_back(location);

    return spiral;
}

std::vector<tf::Vector3> NestedBinVentPlanner::makeLawnmower(const tf::Vector3& startLocation,
                                                    double alongTrackDirection,
                                                    double acrossTrackDirection,
                                                    double alongTrackSize,
                                                    double acrossTrackSize,
                                                    double spacing)
{
    std::vector<tf::Vector3> lawnmower;
    lawnmower.push_back(startLocation);

    tf::Vector3 location = startLocation;
    const std::vector<double> directions = {alongTrackDirection, 
                                            acrossTrackDirection, 
                                            alongTrackDirection - M_PI, 
                                            acrossTrackDirection};

    const std::vector<double> distance = {alongTrackSize, 
                                          spacing, 
                                          alongTrackSize, 
                                          spacing};                   

    

    unsigned legIndex = 0;
    unsigned trackIndex = 0;
    while(spacing * trackIndex <= acrossTrackSize)
    {
        //Transect1 at transectLength
        location.setX(location.getX() + (cos(directions[legIndex]) * distance[legIndex]));
        location.setY(location.getY() + (sin(directions[legIndex]) * distance[legIndex]));
        location.setZ(startLocation.getZ());
        lawnmower.push_back(location);

        if(legIndex == 0 || legIndex == 2)
        {
            trackIndex++;
        }
        legIndex = (legIndex + 1) % directions.size();

        
    }

    return lawnmower;
}