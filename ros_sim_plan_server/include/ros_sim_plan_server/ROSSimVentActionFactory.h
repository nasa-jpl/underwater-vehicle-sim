#ifndef ROS_SIM_VENT_ACTION_FACTORY_H
#define ROS_SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/ChargeSimActionExecutor.h"
#include "ros_sim_plan_server/action_executors/DataTransferSimActionExecutor.h"

class ROSSimVentActionFactory : public VentActionFactory
{
public:
    ROSSimVentActionFactory(ros::NodeHandle& nh);
    ~ROSSimVentActionFactory() {}

    std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                           const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const double upperDepth,
                                                           const double lowerDepth,
                                                           const std::vector<tf::Vector3>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;

    std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                           const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const std::vector<tf::Vector3>& points,
                                                           const PointPathAction::ReplanType replan,
                                                           const double periodicReplanTime) override;

    std::shared_ptr<ChargeAction> createChargeAction(const std::string& vehicleName) override;
    std::shared_ptr<DataTransferAction> createDataTransferAction(const std::string& vehicleName) override;

    std::shared_ptr<DynamicLawnmowerAction> createDynamicLawnmowerAction(const std::string& vehicleName,
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
                                                                         const int trackSectionThreshold) override;
private:
    ros::NodeHandle& nh;
};

#endif
