#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/NestedBinVentPlanner.h"

#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeData.h"
#include "plume_detector/GetPlumeData.h"

NestedBinVentPlanner::NestedBinVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    lastPlan(ros::Time::now()),
    initalPlan(false),
    plumeHeight(0),
    vehicleName(vehicleName),
    dataClient(nh.serviceClient<data_server::GetData>("/data_server/get")),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("/data_server/get_latest")),
    plumeClient(nh.serviceClient<plume_detector::GetPlumeData>("/plume_detector/get")),
    spiralData(tf::Vector3(0,0,0), 200000, 0)
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
}

std::shared_ptr<Plan> NestedBinVentPlanner::plan()
{
    ROS_INFO("Planner: Plan Start");
    //Pop the top plan if it has been completed
    if(plans.size() > 0 && isCompleted(plans.top()))
    {
        ROS_INFO("Planner: Finished Plan");
        plans.pop();

        if(plans.size() > 0)
        {
            ROS_INFO("Planner: Restart previous plan");
            lastPlan = ros::Time::now();
            plans.top()->resetInterrupted();
            return plans.top();
        }
    }
    else
    {
        ROS_INFO("Planner: No Finished Plan");
    }

    //Create inital plan and push it onto the stack
    if(!initalPlan && plans.size() == 0)
    {   
        std::shared_ptr<Plan> plan(new Plan());

        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            ROS_INFO("Planner: Generate Inital Plan");
            tf::Vector3 vehicleLocation(latestEntry.x, latestEntry.y, latestEntry.h);
            std::vector<tf::Vector3> spiralPoints = makeSpiral(vehicleLocation, 0, initalSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                    1.0,
                                                                                    0.349066,
                                                                                    0.523599, //30 deg
                                                                                    -100,
                                                                                    -2000,
                                                                                    spiralPoints);
            plan->addAction(newAction);

            //Add new plan to stack of plans
            plans.push(plan);
           
            initalPlan = true;

            lastPlan = ros::Time::now();
            ROS_INFO("Planner: Plan Generated");
            return plan;
        }
    }
    else
    {
        ROS_INFO("Planner: Plan");
        plume_detector::GetPlumeData srv;
        srv.request.name = vehicleName;
        srv.request.start_time = lastPlan;
        srv.request.end_time = ros::Time::now();

        ROS_INFO("Planner: Get plume data");
        plumeClient.call(srv);

        if(plans.size() == 1) //If on spiral, save data to the spiralData bin 
        {

            ROS_INFO("Planner: Add spiral plume data; Data Size: %lu", srv.response.time.size());
            spiralData.clear(); //Clear data so we can easily calculate plume height and mav value
            for(unsigned int i = 0; i < srv.response.time.size(); i++)
            {
               PlumeData newPlumeData(srv.response.time[i],
                              srv.response.x[i],
                              srv.response.y[i],
                              srv.response.h[i],
                              srv.response.val[i]);
               spiralData.addData(newPlumeData);
            }

            if(spiralData.getMaxVal() >= 0.5)
            {
                tf::Vector3 maxLoc = spiralData.getMaxValLocation();
                double plumeHeight = spiralData.getHeightOfPlume();

                //Create bins
                if(!dataBins)
                {
                    dataBins = std::unique_ptr<DataBins>(new DataBins(spiralData.getMaxValLocation(),
                                                                      150000, 
                                                                      initalSpacing));
                }

                ROS_INFO("Planner: Start Lawnmowers");
                std::shared_ptr<Plan> plan(new Plan());
                
                tf::Vector3 lawnmower0Start(maxLoc.getX() + initalSpacing / 2, maxLoc.getY() + initalSpacing / 2, plumeHeight);
                tf::Vector3 lawnmower1Start(maxLoc.getX() - initalSpacing / 2, maxLoc.getY() + initalSpacing / 2, plumeHeight);
                tf::Vector3 lawnmower2Start(maxLoc.getX() - initalSpacing / 2, maxLoc.getY() - initalSpacing / 2, plumeHeight);
                tf::Vector3 lawnmower3Start(maxLoc.getX() + initalSpacing / 2, maxLoc.getY() - initalSpacing / 2, plumeHeight);

                std::shared_ptr<Action> lawnmower0 = actionFactory->createDynamicLawnmowerAction(vehicleName,
                                                                                                 1.0,
                                                                                                 0.349066,
                                                                                                 0.523599, //30 deg
                                                                                                 lawnmower0Start,
                                                                                                 0,
                                                                                                 M_PI / 2,
                                                                                                 initalSpacing,
                                                                                                 plumeHeight,
                                                                                                 3,
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
                                                                                                 3,
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
                                                                                                 3,
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
                                                                                                 3,
                                                                                                 0.5,
                                                                                                 2);

                plan->addAction(lawnmower0);
                plan->addAction(lawnmower1);
                plan->addAction(lawnmower2);
                plan->addAction(lawnmower3);

                //Add new plan to stack of plans
                plans.push(plan);
           
                lastPlan = ros::Time::now();

                return plan;
            }
        }
        else if(plans.size() >= 1) //Add data to dataBins if we are no longer on the first spiral
        {
            if(dataBins)
            {
                for(unsigned int i = 0; i < srv.response.time.size(); i++)
                {
                   PlumeData newPlumeData(srv.response.time[i],
                                  srv.response.x[i],
                                  srv.response.y[i],
                                  srv.response.h[i],
                                  srv.response.val[i]);
                   dataBins->addData(newPlumeData);
                }
            }
        }
    }

    lastPlan = ros::Time::now();
    if(plans.size() > 0)
    {
        plans.top()->resetInterrupted();
        ROS_INFO("Planner: Top plan sent");
        return plans.top();
    }

     ROS_INFO("Planner: NUll plan sent");
    return nullptr;
}


bool NestedBinVentPlanner::isDone()
{
    return false;
}

bool NestedBinVentPlanner::isCompleted(std::shared_ptr<Plan> plan)
{
    ROS_INFO("Planner: Check for completed");
    for(auto action : plan->getActions())
    {
        if(!(action->getState() == Action::State::COMPLETED || 
             action->getState() == Action::State::FAILED))
        {
            ROS_INFO("Planner: Not Completed: %i", action->getState());
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