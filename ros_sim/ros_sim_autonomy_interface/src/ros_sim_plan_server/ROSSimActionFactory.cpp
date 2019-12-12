#include <unordered_map>
#include <memory>
#include <iostream>
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

std::shared_ptr<PointPathAction> ROSSimActionFactory::createPointPathAction(const std::vector<Eigen::Vector3d>& points,
                                                                            const double targetHorizontalVelocity, 
                                                                            const double targetRotationalVelocity,
                                                                            const double timeout,
                                                                            std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                            const PointPathAction::ReplanType replanType,
                                                                            const double periodicReplanValue)
{
    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(vehicleInfo));
    return std::unique_ptr<PointPathAction>(new PointPathAction(points,
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                timeout,
                                                                std::move(operationRegion),
                                                                replanType,
                                                                periodicReplanValue,
                                                                std::move(executor)));
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

std::shared_ptr<TwoConcurrentActionsAction> ROSSimActionFactory::createTwoConcurrentActionsAction(std::shared_ptr<XYAction> xyAction,
                                                                                                  std::shared_ptr<DepthAction> depthAction,
                                                                                                  std::unique_ptr<OperationRegion> operationRegion,
                                                                                                  const bool onlyEndAtXY)
{

    
        return std::unique_ptr<TwoConcurrentActionsAction>(new TwoConcurrentActionsAction(std::move(xyAction),
                                                                                          std::move(depthAction),
                                                                                          std::move(operationRegion),
                                                                                          onlyEndAtXY));                                                                                  
                                                                                               
}