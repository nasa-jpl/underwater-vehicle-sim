#ifndef VENT_SIM_ACTION_EXECUTOR_H
#define VENT_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "vent_planner/VentActionExecutor.h"
#include "underwater_vehicle_sim/GetVehicleInfo.h"


class VentSimActionExecutor : VentActionExecutor
{
public:
	VentSimActionExecutor(ros::NodeHandle& nh);
	~VentSimActionExecutor() {}

	void executeYoYoPointPathAction(std::string vehicleName, 
									double targetHorizontalVelocity, 
									double targetRotationalVelocity,
									double targetSlope, 
									double minDepth,
									double maxDepth,
									std::vector<tf::Vector3>& points);
	
private:

	bool hasPublisher(std::string topic);

private:
	ros::ServiceClient infoClient;
	std::unordered_map<std::string, underwater_vehicle_sim::GetVehicleInfo::Response> vehicleInfo;
	std::unordered_map<std::string, ros::Publisher> publishers;
};

#endif