#ifndef VENT_SIM_ACTION_EXECUTOR_H
#define VENT_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "vent_planner/VentActionExecutor.h"
#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"


class VentSimActionExecutor : public VentActionExecutor
{
public:
	VentSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	~VentSimActionExecutor() {}

	void executeYoYoPointPathAction(double targetHorizontalVelocity, 
									double targetRotationalVelocity,
									double targetSlope, 
									double uperDepth,
									double lowerDepth,
									std::vector<tf::Vector3>& points);
	
	void monitorYoYoPointPathAction(Action::State& state);
	bool triggerReplanYoYoPointPathAction();
private:

	bool hasPublisher(std::string topic);

private:
	ros::ServiceClient infoClient;
	underwater_vehicle_sim::GetVehicleInfo::Response vehicleInfo;
	std::unordered_map<std::string, ros::Publisher> publishers;

	actionlib::SimpleActionClient<vehicle_auto_control::PointPathAction> pointPathClient;
	vehicle_auto_control::PointPathGoal pointPathGoal;
};

#endif