#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "ros_sim_plan_server/ROSSimActionFactory.h"
#include "underwater_autonomy/planner/actions/ActionFactory.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"

using namespace underwater_autonomy;

ROSSimActionFactory::ROSSimActionFactory(VehicleInfo vehicleInfo) :
    vehicleInfo(vehicleInfo)
{}

std::shared_ptr<FollowHeadingAction> ROSSimActionFactory::createFollowHeadingAction(std::unique_ptr<OperationRegion> operationRegion,
                                                                           const double targetHorizontalVelocity, 
                                                                           const double targetRotationalVelocity,
                                                                           const double heading,
                                                                           const double timeout,
                                                                           const FollowHeadingAction::ReplanType replan,
                                                                           const double periodicReplanTime)
{

    std::unique_ptr<ActionExecutor<FollowHeadingAction>> executor(new FollowHeadingSimActionExecutor(vehicleInfo));
    return std::unique_ptr<FollowHeadingAction>(new FollowHeadingAction(std::move(executor),
                                                                std::move(operationRegion),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                heading,
                                                                timeout,
                                                                replan,
                                                                periodicReplanTime));
}   

std::shared_ptr<PointPathAction> ROSSimActionFactory::createPointPathAction(std::unique_ptr<OperationRegion> operationRegion,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double timeout,
                                                                             const std::vector<Eigen::Vector3d>& points,
                                                                             const PointPathAction::ReplanType replan,
                                                                             const double periodicReplanTime)
{
    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(vehicleInfo));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                std::move(operationRegion),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                timeout,
                                                                points,
                                                                replan,
                                                                periodicReplanTime));
}