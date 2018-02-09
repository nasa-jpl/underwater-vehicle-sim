#ifndef VENT_ACTION_FACTORY_H
#define VENT_ACTION_FACTORY_H

#include <memory>

#include "vent_planner/actions/PointPathAction.h"

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
                                                                   const std::vector<tf::Vector3>& points)=0;

    virtual std::shared_ptr<PointPathAction> createPointPathAction(const std::string& vehicleName,
                                                                   const double targetHorizontalVelocity, 
                                                                   const double targetRotationalVelocity,
                                                                   const double targetSlope,
                                                                   const std::vector<tf::Vector3>& points)=0;
};

#endif