#include "vent_planner/SimVentActionFactory.h"

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vent_planner/PointPathSimActionExecutor.h"
#include "vent_planner/DynamicLawnmowerSimActionExecutor.h"


SimVentActionFactory::SimVentActionFactory(ros::NodeHandle& nh) :
    nh(nh)
{}

std::shared_ptr<PointPathAction> SimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const double upperDepth,
                                                                             const double lowerDepth,
                                                                             const std::vector<tf::Vector3>& points,
                                                                             const bool replan)
{

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                targetSlope,
                                                                upperDepth,
                                                                lowerDepth,
                                                                points,
                                                                replan));
}    



std::shared_ptr<PointPathAction> SimVentActionFactory::createPointPathAction(const std::string& vehicleName,
                                                                             const double targetHorizontalVelocity, 
                                                                             const double targetRotationalVelocity,
                                                                             const double targetSlope,
                                                                             const std::vector<tf::Vector3>& points,
                                                                             const bool replan)
{

    std::unique_ptr<ActionExecutor<PointPathAction>> executor(new PointPathSimActionExecutor(nh, vehicleName));
    return std::unique_ptr<PointPathAction>(new PointPathAction(std::move(executor),
                                                                targetHorizontalVelocity,
                                                                targetRotationalVelocity,
                                                                targetSlope,
                                                                points,
                                                                replan));
}

std::shared_ptr<DynamicLawnmowerAction> SimVentActionFactory::createDynamicLawnmowerAction(const std::string& vehicleName,
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