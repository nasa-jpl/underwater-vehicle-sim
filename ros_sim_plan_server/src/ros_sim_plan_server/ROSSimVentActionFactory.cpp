#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "ros_sim_plan_server/ROSSimVentActionFactory.h"
#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/ChargeSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/DataTransferSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/DynamicLawnmowerSimActionExecutor.h"




ROSSimVentActionFactory::ROSSimVentActionFactory(ros::NodeHandle& nh) :
    nh(nh),
    loopHertz(loopHertz)
{}

std::shared_ptr<PointPathAction> ROSSimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const double upperDepth,
                                                                             const double lowerDepth,
                                                                             const std::vector<tf::Vector3>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
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



std::shared_ptr<PointPathAction> ROSSimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const std::vector<tf::Vector3>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{
    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
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

std::shared_ptr<DynamicLawnmowerAction> ROSSimVentActionFactory::createDynamicLawnmowerAction(const std::string& vehicleName,
                                                                                           const double targetHorizontalVelocity, 
                                                                                           const double targetRotationalVelocity,
                                                                                           const double targetSlope,
                                                                                           const tf::Vector3& startLocation,
                                                                                           const double alongTrackDirection,
                                                                                           const double acrossTrackDirection,
                                                                                           const double trackSpacing,
                                                                                           const double targetHeight,
                                                                                           const int minSectionsPerTrack,
                                                                                           const double continueThreshold,
                                                                                           const int trackSectionThreshold)
{
    std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> executor(new DynamicLawnmowerSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<DynamicLawnmowerAction>(new DynamicLawnmowerAction(std::move(executor),
                                                                              targetHorizontalVelocity, 
                                                                              targetRotationalVelocity,
                                                                              targetSlope,
                                                                              startLocation,
                                                                              alongTrackDirection,
                                                                              acrossTrackDirection,
                                                                              trackSpacing,
                                                                              targetHeight,
                                                                              minSectionsPerTrack,
                                                                              continueThreshold,
                                                                              trackSectionThreshold));
}


std::shared_ptr<ChargeAction> ROSSimVentActionFactory::createChargeAction(const std::string& vehicleName)
{
    std::unique_ptr<ActionExecutor<ChargeAction>> executor(new ChargeSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<ChargeAction>(new ChargeAction(std::move(executor)));
}    

std::shared_ptr<DataTransferAction> ROSSimVentActionFactory::createDataTransferAction(const std::string& vehicleName)
{
    std::unique_ptr<ActionExecutor<DataTransferAction>> executor(new DataTransferSimActionExecutor(nh, vehicleName));

    return std::unique_ptr<DataTransferAction>(new DataTransferAction(std::move(executor)));
}    
