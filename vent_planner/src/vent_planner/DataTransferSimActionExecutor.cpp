#include <vector>
#include <unordered_map>
#include <iostream>

#include "ros/ros.h"

#include "planner_framework/Action.h"

#include "vent_planner/DataTransferSimActionExecutor.h"
#include "vent_planner/actions/DataTransferAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Float64.h"

DataTransferSimActionExecutor::DataTransferSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
    pub = nh.advertise<std_msgs::Float64>("/vehicles/" + vehicleName + "/transferring", 1000);
    sub = nh.subscribe("/vehicles/" + vehicleName + "/dataCapacity", 1, &DataTransferSimActionExecutor::transfer_Remaining_Callback, this);
    transfer_msg.data = 1;
}

DataTransferSimActionExecutor::DataTransferSimActionExecutor(const DataTransferSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
    pub = nh.advertise<std_msgs::Float64>("/vehicles/" + vehicleName + "/transferring", 1000);
    sub = nh.subscribe("/vehicles/" + vehicleName + "/dataCapacity", 1, &DataTransferSimActionExecutor::transfer_Remaining_Callback, this);
    transfer_msg.data = 1;
}

bool DataTransferSimActionExecutor::execute(std::shared_ptr<DataTransferAction> action)
{
    pub.publish(transfer_msg);
	return true;
}

void DataTransferSimActionExecutor::monitor(std::shared_ptr<DataTransferAction> action)
{
    pub.publish(transfer_msg);
}

void DataTransferSimActionExecutor::transfer_Remaining_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    transfer_left = msg->data;
}

std::unique_ptr<ActionExecutor<DataTransferAction>> DataTransferSimActionExecutor::clone()
{
    std::unique_ptr<ActionExecutor<DataTransferAction>> a(new DataTransferSimActionExecutor(*this));
    return a;
}