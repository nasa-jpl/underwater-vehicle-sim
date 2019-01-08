#ifndef ROS_SIM_VENT_ACTION_FACTORY_H
#define ROS_SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "underwater_util/VehiclePose.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/ChargeSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/DataTransferSimActionExecutor.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

class ROSSimVentActionFactory : public VentActionFactory
{
public:
    ROSSimVentActionFactory(ros::NodeHandle& nh, VehicleInfo vehicleInfo);
    ~ROSSimVentActionFactory() {}

    std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const double upperDepth,
                                                           const double lowerDepth,
                                                           const std::vector<VehiclePose>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;

    std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const std::vector<VehiclePose>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;

    std::shared_ptr<ChargeAction> createChargeAction() override;
    std::shared_ptr<DataTransferAction> createDataTransferAction() override;
private:
    ros::NodeHandle& nh;
    VehicleInfo vehicleInfo;
};

#endif
