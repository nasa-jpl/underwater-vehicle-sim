#ifndef SIM_VENT_ACTION_FACTORY_H
#define SIM_VENT_ACTION_FACTORY_H

#include <unordered_map>
#include <memory>

#include "ros/ros.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/actions/YoYoPointPathAction.h"
#include "vent_planner/YoYoPointPathSimActionExecutor.h"
class SimVentActionFactory : public VentActionFactory
{
public:
	SimVentActionFactory(ros::NodeHandle& nh);
	~SimVentActionFactory() {}

	std::unique_ptr<YoYoPointPathAction> createYoYoPointPathAction(const std::string& vehicleName,
															       const double targetHorizontalVelocity, 
															       const double targetRotationalVelocity,
															       const double targetSlope,
															       const double upperDepth,
															       const double lowerDepth,
															       const std::vector<tf::Vector3>& points) override;

private:
	ros::NodeHandle& nh;
	std::unordered_map<std::string, YoYoPointPathSimActionExecutor> yoyoPointPathExecutors;

};

#endif