#ifndef CHARGE_SIM_ACTION_EXECUTOR_H
#define CHARGE_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>
#include <memory>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/ChargeAction.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"


class ChargeSimActionExecutor : public ActionExecutor<ChargeAction>
{
public:
	ChargeSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	ChargeSimActionExecutor(const ChargeSimActionExecutor& other);
	~ChargeSimActionExecutor() {}

	/**
	* Executes the yoyo action in the ros simulation with the given parameters
	*/
	bool execute(std::shared_ptr<ChargeAction> action) override;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	* All monitoring is done with action callbacks so this method is not used here
	*/
	void monitor(std::shared_ptr<ChargeAction> action) override; 

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	bool triggerReplan(std::shared_ptr<ChargeAction> action) {}

	void cancel(std::shared_ptr<ChargeAction> action) {}

    void charge_Remaining_Callback(const std_msgs::Float64::ConstPtr& msg);

    std::unique_ptr<ActionExecutor<ChargeAction>> clone();


private:
	ros::NodeHandle& nh;
	ros::ServiceClient infoClient;
	underwater_vehicle_msgs::GetVehicleInfo::Response vehicleInfo;

	std::string vehicleName;
    std_msgs::Float64 charge_msg;
    ros::Publisher pub;
    ros::Subscriber sub;
    double charging_left;

};

#endif
