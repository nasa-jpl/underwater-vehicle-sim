#include "vent_planner/SimVentActionFactory.h"

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/YoYoPointPathAction.h"
#include "vent_planner/actions/ChargeAction.h"

SimVentActionFactory::SimVentActionFactory(ros::NodeHandle& nh) :
	nh(nh)
{}

std::shared_ptr<YoYoPointPathAction> SimVentActionFactory::createYoYoPointPathAction(const std::string& vehicleName,
																				     const double targetHorizontalVelocity, 
																				     const double targetRotationalVelocity,
																				     const double targetSlope,
																				     const double upperDepth,
																				     const double lowerDepth,
																				     const std::vector<tf::Vector3>& points)
{

	if(yoyoPointPathExecutors.find(vehicleName) == yoyoPointPathExecutors.end())
	{
		yoyoPointPathExecutors.insert(std::make_pair(vehicleName, YoYoPointPathSimActionExecutor(nh, vehicleName)));
	}

	auto executor = yoyoPointPathExecutors.find(vehicleName);
	return std::unique_ptr<YoYoPointPathAction>(new YoYoPointPathAction(executor->second,
																        targetHorizontalVelocity,
																        targetRotationalVelocity,
																        targetSlope,
																        upperDepth,
																        lowerDepth,
																        points));
}    

std::shared_ptr<ChargeAction> SimVentActionFactory::createChargeAction(const std::string& vehicleName)
{

	if(chargeExecutors.find(vehicleName) == chargeExecutors.end())
	{
		chargeExecutors.insert(std::make_pair(vehicleName, ChargeSimActionExecutor(nh, vehicleName)));
	}

	auto executor = chargeExecutors.find(vehicleName);
	return std::unique_ptr<ChargeAction>(new ChargeAction(executor->second));
}    
