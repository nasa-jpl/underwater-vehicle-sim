#include "vent_planner/SimVentActionFactory.h"

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"

SimVentActionFactory::SimVentActionFactory(ros::NodeHandle& nh) :
    nh(nh)
{}

std::shared_ptr<PointPathAction> SimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const double upperDepth,
                                                                             const double lowerDepth,
                                                                             const std::vector<tf::Vector3>& points)
{

    if(pointPathExecutors.find(vehicleName) == pointPathExecutors.end())
    {
        pointPathExecutors.insert(std::make_pair(vehicleName, PointPathSimActionExecutor(nh, vehicleName)));
    }

    auto executor = pointPathExecutors.find(vehicleName);
    return std::unique_ptr<PointPathAction>(new PointPathAction(executor->second,
                                                                        targetHorizontalVelocity,
                                                                        targetRotationalVelocity,
                                                                        targetSlope,
                                                                        upperDepth,
                                                                        lowerDepth,
                                                                        points));
}    



std::shared_ptr<PointPathAction> SimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const std::vector<tf::Vector3>& points)
{

    if(pointPathExecutors.find(vehicleName) == pointPathExecutors.end())
    {
        pointPathExecutors.insert(std::make_pair(vehicleName, PointPathSimActionExecutor(nh, vehicleName)));
    }

    auto executor = pointPathExecutors.find(vehicleName);
    return std::unique_ptr<PointPathAction>(new PointPathAction(executor->second,
                                                                        targetHorizontalVelocity,
                                                                        targetRotationalVelocity,
                                                                        targetSlope,
                                                                        points));
}    