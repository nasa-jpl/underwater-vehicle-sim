#ifndef ROS_SIM_VENT_ACTION_FACTORY_H
#define ROS_SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include "underwater_autonomy/util/VehiclePose.h"
#include "underwater_autonomy/util/OperationRegion.h"

#include "underwater_autonomy/planner/actions/ActionFactory.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"
#include "underwater_autonomy/planner/actions/FollowHeadingAction.h"
#include "underwater_autonomy/planner/actions/HoldDepthAction.h"
#include "underwater_autonomy/planner/actions/YoYoAction.h"
#include "underwater_autonomy/planner/actions/CombinedMoveAction.h"
#include "underwater_autonomy/planner/actions/XYAction.h"
#include "underwater_autonomy/planner/actions/DepthAction.h"


#include "underwater_vehicle_msgs/VehicleInfo.h"

class ROSSimActionFactory : public underwater_autonomy::ActionFactory
{
public:
    ROSSimActionFactory(VehicleInfo vehicleInfo);
    ~ROSSimActionFactory() {}

    std::shared_ptr<underwater_autonomy::YoYoAction> createYoYoAction(const double upperDepth,
                                                                      const double lowerDepth,
                                                                      const double targetVerticalVelocity, 
                                                                      const double yoyoTime,
                                                                      const double timeout,
                                                                      std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                      const underwater_autonomy::YoYoAction::ReplanType replanType,
                                                                      const double periodicReplanValue) override;

    std::shared_ptr<underwater_autonomy::HoldDepthAction> createHoldDepthAction(const double depth,
                                                                                const double targetVerticalVelocity,
                                                                                const double holdDepthTime,
                                                                                const double timeout,
                                                                                std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                const underwater_autonomy::HoldDepthAction::ReplanType replanType,
                                                                                const double periodicReplanValue) override;

    std::shared_ptr<underwater_autonomy::FollowHeadingAction> createFollowHeadingAction(const double heading,
                                                                                        const double targetHorizontalVelocity, 
                                                                                        const double targetRotationalVelocity,
                                                                                        const double followHeadingTime,
                                                                                        const double timeout,
                                                                                        std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                        const underwater_autonomy::FollowHeadingAction::ReplanType replanType,
                                                                                        const double periodicReplanValue) override;

    std::shared_ptr<underwater_autonomy::PointPathAction> createPointPathAction(const std::vector<Eigen::Vector3d>& points,
                                                                                const double targetHorizontalVelocity, 
                                                                                const double targetRotationalVelocity,
                                                                                const double timeout,
                                                                                std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                const underwater_autonomy::PointPathAction::ReplanType replanType,
                                                                                const double periodicReplanValue) override;

    std::shared_ptr<underwater_autonomy::CombinedMoveAction> createCombinedMoveAction(std::shared_ptr<underwater_autonomy::XYAction> xyAction,
                                                                                                      std::shared_ptr<underwater_autonomy::DepthAction> depthAction,
                                                                                                      std::unique_ptr<underwater_autonomy::OperationRegion> operationRegion,
                                                                                                      const bool onlyEndAtXY = false) override;
private:
    VehicleInfo vehicleInfo;
};

#endif