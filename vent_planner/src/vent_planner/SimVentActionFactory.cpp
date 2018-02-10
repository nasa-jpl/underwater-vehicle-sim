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

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
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

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                targetSlope,
                                                                points));
}    