#ifndef VENT_ACTION_EXECUTOR_H
#define VENT_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Action.h"
#include "underwater_vehicle_sim/GetVehicleInfo.h"


class VentActionExecutor
{
public:
	VentActionExecutor(ros::NodeHandle& nh, std::string vehicleName) : 
					   nh(nh), vehicleName(vehicleName) {};

	virtual ~VentActionExecutor() {}

	virtual void executeYoYoPointPathAction(double targetHorizontalVelocity, 
											double targetRotationalVelocity, 
											double targetSlope, 
											double upperDepth,
											double lowerDepth,
											std::vector<tf::Vector3>& points)=0;

	virtual void monitorYoYoPointPathAction(Action::State& state)=0;
	virtual bool triggerReplanYoYoPointPathAction()=0;
	
protected:
	ros::NodeHandle& nh;
	std::string vehicleName;
};

#endif