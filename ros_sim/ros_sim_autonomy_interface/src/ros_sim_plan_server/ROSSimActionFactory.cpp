#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "ros_sim_plan_server/ROSSimActionFactory.h"
#include "underwater_autonomy/planner/actions/ActionFactory.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/FollowHeadingSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/HoldDepthSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/YoYoSimActionExecutor.h"

using namespace underwater_autonomy;

ROSSimActionFactory::ROSSimActionFactory(VehicleInfo vehicleInfo) :
    vehicleInfo(vehicleInfo)
{}

std::shared_ptr<underwater_autonomy::YoYoAction> ROSSimActionFactory::createYoYoAction(const double upperDepth,
                                                                                       const double lowerDepth,
                                                                                       const double targetVerticalVelocity, 
                                                                                       const double yoyoTime,
                                                                                       const double timeout,
                                                                                       std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                       const YoYoAction::ReplanType replanType,
                                                                                       const double periodicReplanValue)
{
    std::unique_ptr<ActionExecutor<YoYoAction>> executor(new YoYoSimActionExecutor(vehicleInfo));
    return std::unique_ptr<YoYoAction>(new YoYoAction(upperDepth,
                                                      lowerDepth,
                                                      targetVerticalVelocity,
                                                      yoyoTime,
                                                      timeout,
                                                      std::move(operationRegion),
                                                      replanType,
                                                      periodicReplanValue,
                                                      std::move(executor)));

}

std::shared_ptr<FollowHeadingAction> ROSSimActionFactory::createFollowHeadingAction(const double heading,
                                                                                    const double targetHorizontalVelocity, 
                                                                                    const double targetRotationalVelocity,
                                                                                    const double followHeadingTime,
                                                                                    const double timeout,
                                                                                    std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                    const FollowHeadingAction::ReplanType replanType,
                                                                                    const double periodicReplanValue)
{

    std::unique_ptr<ActionExecutor<FollowHeadingAction>> executor(new FollowHeadingSimActionExecutor(vehicleInfo));
    return std::unique_ptr<FollowHeadingAction>(new FollowHeadingAction(heading,
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                followHeadingTime,
                                                                timeout,
                                                                std::move(operationRegion),
                                                                replanType,
                                                                periodicReplanValue,
                                                                std::move(executor)));
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

std::shared_ptr<HoldDepthAction> ROSSimActionFactory::createHoldDepthAction(const double depth,
                                                                            const double targetVerticalVelocity,
                                                                            const double holdDepthTime,
                                                                            const double timeout,
                                                                            std::unique_ptr<OperationRegion> operationRegion,
                                                                            const HoldDepthAction::ReplanType replanType,
                                                                            const double periodicReplanValue)
{
    std::unique_ptr<ActionExecutor<HoldDepthAction>> executor(new HoldDepthSimActionExecutor(vehicleInfo));
    return std::unique_ptr<HoldDepthAction>(new HoldDepthAction(depth,
                                                                targetVerticalVelocity,
                                                                holdDepthTime,
                                                                timeout,
                                                                std::move(operationRegion),
                                                                replanType,
                                                                periodicReplanValue,
                                                                std::move(executor)));
}

std::shared_ptr<TwoConcurrentActionsAction> ROSSimActionFactory::createTwoConcurrentActionsAction(std::shared_ptr<xyAction> xy_action,
                                                                                                  std::shared_ptr<DepthAction> depth_action,
                                                                                                  std::unique_ptr<OperationRegion> operationRegion)
{

    // need to find which type of action the xy and depth actions are in order to use executors
    

    // if (dynamic_cast<FollowHeadingAction*>(&xy_action) != nullptr) {
    //   std::unique_ptr<ActionExecutor<FollowHeadingAction>> xy_executor(new FollowHeadingSimActionExecutor(vehicleInfo));
    // }
    // if (dynamic_cast<PointPathAction*>(&xy_action) != nullptr) {
    //   std::unique_ptr<ActionExecutor<FollowHeadingAction>> xy_executor(new PointPathSimActionExecutor(vehicleInfo));
    // }

    // if (dynamic_cast<YoYoAction*>(depth_action) != nullptr) {
    //   std::unique_ptr<ActionExecutor<YoYoAction>> depth_executor(new YoYoSimActionExecutor(vehicleInfo));
    // }

    // else if (dynamic_cast<HoldDepthAction*>(depth_action) != nullptr) {
    //   std::unique_ptr<ActionExecutor<HoldDepthAction>> depth_executor(new HoldDepthSimActionExecutor(vehicleInfo));
    // }

    // return std::unique_ptr<TwoConcurrentActionsAction>(new TwoConcurrentActionsAction(std::move(xy_action),
    //                                                                                   std::move(depth_action),
    //                                                                                   std::move(xy_executor),
    //                                                                                   std::move(depth_executor),
    //                                                                                   std::move(operationRegion)
    
        return std::unique_ptr<TwoConcurrentActionsAction>(new TwoConcurrentActionsAction(std::move(xy_action),
                                                                                          std::move(depth_action),
                                                                                          std::move(operationRegion)));                                                                                  
                                                                                               
}