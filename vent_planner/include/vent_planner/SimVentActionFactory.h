#ifndef SIM_VENT_ACTION_FACTORY_H
#define SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"

class SimVentActionFactory : public VentActionFactory
{
public:
    SimVentActionFactory(ros::NodeHandle& nh);
    ~SimVentActionFactory() {}

    std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                           const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const double upperDepth,
                                                           const double lowerDepth,
                                                           const std::vector<tf::Vector3>& points) override;

    std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                           const double targetHorizontalVelocity, 
                                                           const double targetRotationalVelocity,
                                                           const double targetSlope,
                                                           const std::vector<tf::Vector3>& points) override;

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