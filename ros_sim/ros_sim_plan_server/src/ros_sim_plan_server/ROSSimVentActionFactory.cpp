#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "ros_sim_plan_server/ROSSimVentActionFactory.h"
#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/ChargeSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/DataTransferSimActionExecutor.h"


ROSSimVentActionFactory::ROSSimVentActionFactory(ros::NodeHandle& nh, VehicleInfo vehicleInfo) :
    nh(nh),
    vehicleInfo(vehicleInfo)
{}

std::shared_ptr<PointPathAction> ROSSimVentActionFactory::createPointPathAction(const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const double upperDepth,
                                                                             const double lowerDepth,
                                                                             const std::vector<VehiclePose>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleInfo.getName()));
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
                                                                             const std::vector<VehiclePose>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{
    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleInfo.getName()));
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

std::shared_ptr<ChargeAction> ROSSimVentActionFactory::createChargeAction()
{
    std::unique_ptr<ActionExecutor<ChargeAction>> executor(new ChargeSimActionExecutor(nh, vehicleInfo.getName()));
    return std::unique_ptr<ChargeAction>(new ChargeAction(std::move(executor)));
}    

std::shared_ptr<DataTransferAction> ROSSimVentActionFactory::createDataTransferAction()
{
    std::unique_ptr<ActionExecutor<DataTransferAction>> executor(new DataTransferSimActionExecutor(nh, vehicleInfo.getName()));

    return std::unique_ptr<DataTransferAction>(new DataTransferAction(std::move(executor)));
}    
