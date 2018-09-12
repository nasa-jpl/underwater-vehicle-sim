#include <vector>
#include <unordered_map>
#include <iostream>

#include "ros/ros.h"

#include "planner_framework/Action.h"

#include "vent_planner/executors/ChargeSimActionExecutor.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Float64.h"

ChargeSimActionExecutor::ChargeSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
    pub = nh.advertise<std_msgs::Float64>("vehicles/" + vehicleName + "charging", 1000);
    sub = nh.subscribe("vehicles/" + vehicleName + "/power", 1, &ChargeSimActionExecutor::charge_Remaining_Callback, this);
    charge_msg.data = 1;
}

ChargeSimActionExecutor::ChargeSimActionExecutor(const ChargeSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
    pub = nh.advertise<std_msgs::Float64>("vehicles/" + vehicleName + "/charging", 1000);
    sub = nh.subscribe("vehicles/" + vehicleName + "/power", 1, &ChargeSimActionExecutor::charge_Remaining_Callback, this);
    charge_msg.data = 1;
}

bool ChargeSimActionExecutor::execute(std::shared_ptr<ChargeAction> action)
{
    pub.publish(charge_msg);
	return true;
}

void ChargeSimActionExecutor::monitor(std::shared_ptr<ChargeAction> action)
{
    pub.publish(charge_msg);
}

void ChargeSimActionExecutor::charge_Remaining_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    charging_left = msg->data;
}

std::unique_ptr<ActionExecutor<ChargeAction>> ChargeSimActionExecutor::clone()
{
    std::unique_ptr<ActionExecutor<ChargeAction>> a(new ChargeSimActionExecutor(*this));
    return a;
}