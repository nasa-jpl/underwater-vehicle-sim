#ifndef ROS_SIM_VENT_ACTION_FACTORY_H
#define ROS_SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include "underwater_util/VehiclePose.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

class ROSSimVentActionFactory : public VentActionFactory
{
public:
    ROSSimVentActionFactory(VehicleInfo vehicleInfo);
    ~ROSSimVentActionFactory() {}

    std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const double upperDepth,
                                                           const double lowerDepth,
                                                           const std::vector<Eigen::Vector3d>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;

    std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const std::vector<Eigen::Vector3d>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;
private:
    VehicleInfo vehicleInfo;
};

#endif
