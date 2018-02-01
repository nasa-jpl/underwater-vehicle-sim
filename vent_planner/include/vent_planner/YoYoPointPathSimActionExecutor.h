#ifndef VENT_SIM_ACTION_EXECUTOR_H
#define VENT_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/YoYoPointPathAction.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"


class YoYoPointPathSimActionExecutor : public ActionExecutor<YoYoPointPathAction>
{
public:
	YoYoPointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	YoYoPointPathSimActionExecutor(const YoYoPointPathSimActionExecutor& other);
	~YoYoPointPathSimActionExecutor() {}

	/**
	* Executes the yoyo action in the ros simulation with the given parameters
	*/
	bool execute(YoYoPointPathAction& action) override;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	*/
	void monitor(YoYoPointPathAction& action) override;

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	bool triggerReplan(YoYoPointPathAction& action) override;
private:

	bool hasPublisher(std::string topic);

private:
	ros::NodeHandle& nh;
	ros::ServiceClient infoClient;
	underwater_vehicle_sim::GetVehicleInfo::Response vehicleInfo;
	std::unordered_map<std::string, ros::Publisher> publishers;

	std::string vehicleName;

	actionlib::SimpleActionClient<vehicle_auto_control::PointPathAction> pointPathClient;
	vehicle_auto_control::PointPathGoal pointPathGoal;
};

#endif