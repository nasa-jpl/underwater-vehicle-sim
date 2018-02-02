#include "vent_planner/SimVentActionFactory.h"

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/YoYoPointPathAction.h"

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