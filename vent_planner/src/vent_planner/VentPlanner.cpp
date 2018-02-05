#include <vector>
#include <memory>
#include <math.h>
#include <limits>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/VentPlanner.h"

#include "data_server/DataServerEntry.h"

VentPlanner::VentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
 nh(nh),
 actionFactory(std::move(actionFactory)),
 lastPlan(ros::Time::now()),
 initalPlan(false),
 initalSpacing(3000),
 vehicleName(vehicleName),
 dataClient(nh.serviceClient<data_server::GetData>("/data_server/get")),
 latestDataClient(nh.serviceClient<data_server::GetLatestData>("/data_server/get_latest"))
{
    ROS_INFO("Waiting for data server...");
    dataClient.waitForExistence();
    latestDataClient.waitForExistence();
}

std::shared_ptr<Plan> VentPlanner::plan()
{
    //Pop the top plan if it has been completed
    if(plans.size() > 0 && isCompleted(plans.top()))
    {
        plans.pop();
    }

    //Create inital plan and push it onto the stack
    if(!initalPlan && plans.size() == 0)
    {
        initalPlan = true;
        std::shared_ptr<Plan> plan(new Plan());

        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            tf::Vector3 vehicleLocation(latestEntry.x, latestEntry.y, latestEntry.h);
            std::vector<tf::Vector3> spiralPoints = makeSpiral(vehicleLocation, 0, initalSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createYoYoPointPathAction(vehicleName,
                                                                                    1.0,
                                                                                    0.349066,
                                                                                    0.523599, //30 deg
                                                                                    -100,
                                                                                    -2000,
                                                                                    spiralPoints); 
            plan->addAction(newAction);

            ROS_INFO("Created inital plan.");
            return plan;
        }
    }

    return nullptr;
}

bool VentPlanner::isCompleted(std::shared_ptr<Plan> plan)
{
    for(auto action : plan->getActions())
    {
        if(!(action->getState() == Action::State::COMPLETED || 
             action->getState() == Action::State::FAILED))
        {
            return false;
        }
    }
    return true;
}

bool VentPlanner::getLatestData(DataServerEntry& returnEntry)
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

std::vector<tf::Vector3> VentPlanner::makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size)
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
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        //Transect2 at transectLength
        location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
        location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        lengthIndex++;
    }

    //Final transect to finish out the spiral, same transect length as the last segment
    location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    spiral.push_back(location);

    return spiral;
}

std::vector<tf::Vector3> VentPlanner::makeLawnmower(const tf::Vector3& startLocation,
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
        lawnmower.push_back(location);

        if(legIndex == 0 || legIndex == 2)
        {
            trackIndex++;
        }
        legIndex = (legIndex + 1) % directions.size();

        
    }

    return lawnmower;
}