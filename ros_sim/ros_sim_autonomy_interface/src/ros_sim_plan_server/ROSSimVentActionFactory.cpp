#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "ros_sim_plan_server/ROSSimVentActionFactory.h"
#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"

ROSSimVentActionFactory::ROSSimVentActionFactory(VehicleInfo vehicleInfo) :
    vehicleInfo(vehicleInfo)
{}

std::shared_ptr<PointPathAction> ROSSimVentActionFactory::createPointPathAction(const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const double upperDepth,
                                                                             const double lowerDepth,
                                                                             const std::vector<Eigen::Vector3d>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(vehicleInfo));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                targetSlope,
                                                                true,
                                                                upperDepth,
                                                                lowerDepth,
                                                                points,
                                                                replan,
                                                                periodicReplanTime));
}    

std::shared_ptr<PointPathAction> ROSSimVentActionFactory::createPointPathAction(const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const std::vector<Eigen::Vector3d>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{
    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(vehicleInfo));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                targetSlope,
                                                                false,
                                                                0,
                                                                0,
                                                                points,
                                                                replan,
                                                                periodicReplanTime));
}