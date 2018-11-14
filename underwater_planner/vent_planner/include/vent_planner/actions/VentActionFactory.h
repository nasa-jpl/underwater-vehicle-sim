#ifndef VENT_ACTION_FACTORY_H
#define VENT_ACTION_FACTORY_H

#include <memory>

#include "planner_framework/VehiclePose.h"

#include "vent_planner/actions/PointPathAction.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vent_planner/actions/DataTransferAction.h"

class VentActionFactory
{
public:
    VentActionFactory() {}
    virtual ~VentActionFactory() {}

    virtual std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                                   const double targetRotationalVelocity,
                                                                   const double targetSlope,
                                                                   const double upperDepth,
                                                                   const double lowerDepth,
                                                                   const std::vector<VehiclePose>& points,
                                                                   const PointPathAction::ReplanType replan,
                                                                   const double periodicReplanTime)=0;

    virtual std::shared_ptr<PointPathAction> createPointPathAction(const double targetHorizontalVelocity, 
                                                                   const double targetRotationalVelocity,
                                                                   const double targetSlope,
                                                                   const std::vector<VehiclePose>& points,
                                                                   const PointPathAction::ReplanType replan,
                                                                   const double periodicReplanTime)=0;

    virtual std::shared_ptr<ChargeAction> createChargeAction()=0;
    virtual std::shared_ptr<DataTransferAction> createDataTransferAction()=0;
};

#endif
