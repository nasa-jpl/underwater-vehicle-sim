#ifndef VENT_ACTION_FACTORY_H
#define VENT_ACTION_FACTORY_H

#include <memory>

#include "vent_planner/actions/YoYoPointPathAction.h"
#include "vent_planner/actions/ChargeAction.h"

class VentActionFactory
{
public:
	VentActionFactory() {}
	virtual ~VentActionFactory() {}

	virtual std::shared_ptr<YoYoPointPathAction> createYoYoPointPathAction(const std::string& vehicleName,
																	   	   const double targetHorizontalVelocity, 
															   		       const double targetRotationalVelocity,
															   		   	   const double targetSlope,
															   		   	   const double upperDepth,
															   		   	   const double lowerDepth,
															   		   	   const std::vector<tf::Vector3>& points)=0;
	
    virtual std::shared_ptr<ChargeAction> createChargeAction(const std::string& vehicleName)=0;
};

#endif
