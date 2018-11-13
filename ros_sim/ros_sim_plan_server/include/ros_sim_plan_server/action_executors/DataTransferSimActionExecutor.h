#ifndef DATA_SIM_ACTION_EXECUTOR_H
#define DATA_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/DataTransferAction.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"


class DataTransferSimActionExecutor : public ActionExecutor<DataTransferAction>
{
public:
	DataTransferSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	DataTransferSimActionExecutor(const DataTransferSimActionExecutor& other);
	~DataTransferSimActionExecutor() {}

	/**
	* Executes the yoyo action in the ros simulation with the given parameters
	*/
	bool execute(std::shared_ptr<DataTransferAction> action) override;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	* All monitoring is done with action callbacks so this method is not used here
	*/
	void monitor(std::shared_ptr<DataTransferAction> action) override; 

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	bool triggerReplan(std::shared_ptr<DataTransferAction> action) override;

	void cancel(std::shared_ptr<DataTransferAction> action) override {}

    void transfer_Remaining_Callback(const std_msgs::Float64::ConstPtr& msg);

    std::unique_ptr<ActionExecutor<DataTransferAction>> clone() override;
private:
	ros::NodeHandle& nh;
	ros::ServiceClient infoClient;
	underwater_vehicle_msgs::GetVehicleInfo::Response vehicleInfo;

	std::string vehicleName;
    std_msgs::Float64 transfer_msg;
    ros::Publisher pub;
    ros::Subscriber sub;
    double transfer_left;

};

#endif
