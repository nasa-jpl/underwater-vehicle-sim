#ifndef VENT_ACTION_FACTORY_H
#define VENT_ACTION_FACTORY_H

#include <memory>

#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

class VentActionFactory
{
public:
    VentActionFactory() {}
    virtual ~VentActionFactory() {}

    virtual std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                                   const double targetHorizontalVelocity, 
                                                                   const double targetRotationalVelocity,
                                                                   const double targetSlope,
                                                                   const double upperDepth,
                                                                   const double lowerDepth,
                                                                   const std::vector<tf::Vector3>& points,
                                                                   const bool replan)=0;

    virtual std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                                   const double targetHorizontalVelocity, 
                                                                   const double targetRotationalVelocity,
                                                                   const double targetSlope,
                                                                   const std::vector<tf::Vector3>& points,
                                                                   const bool replan)=0;

    virtual std::shared_ptr<DynamicLawnmowerAction> createDynamicLawnmowerAction(const std::string& vehicleName,
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
                                                                                 const int trackSectionThreshold)=0;

    virtual std::shared_ptr<ChargeAction> createChargeAction(const std::string& vehicleName)=0;
    virtual std::shared_ptr<DataTransferAction> createDataTransferAction(const std::string& vehicleName)=0;
};

#endif
