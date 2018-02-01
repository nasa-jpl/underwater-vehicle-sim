#ifndef VENT_ACTION_FACTORY_H
#define VENT_ACTION_FACTORY_H

#include <memory>

#include "vent_planner/actions/YoYoPointPathAction.h"

class VentActionFactory
{
public:
	VentActionFactory() {}
	virtual ~VentActionFactory() {}

	virtual std::unique_ptr<YoYoPointPathAction> createYoYoPointPathAction(const std::string& vehicleName,
																	   	   const double targetHorizontalVelocity, 
															   		       const double targetRotationalVelocity,
															   		   	   const double targetSlope,
															   		   	   const double upperDepth,
															   		   	   const double lowerDepth,
															   		   	   const std::vector<tf::Vector3>& points)=0;
};

#endif