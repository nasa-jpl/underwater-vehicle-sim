#ifndef VENT_ACTION_EXECUTOR_H
#define VENT_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"


class VentActionExecutor
{
public:
	VentActionExecutor(ros::NodeHandle& nh) : nh(nh) {};
	virtual ~VentActionExecutor() {}

	virtual void executeYoYoPointPathAction(std::string vehicleName, 
											double targetHorizontalVelocity, 
											double targetRotationalVelocity, 
											double targetSlope, 
											double minDepth,
											double maxDepth,
											std::vector<tf::Vector3>& points)=0;
	
protected:
	ros::NodeHandle& nh;
};

#endif