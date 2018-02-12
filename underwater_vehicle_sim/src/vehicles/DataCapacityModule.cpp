#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "vehicles/DataCapacityModule.h"

#define SECONDS_IN_DAY 86400

DataCapacityModule::DataCapacityModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "DataCapacity", parentNH, vehicleName)
{
	sentData = 0.0;
    inBaseRange = false;
    nh.getParam("/transfer_rate", transferRate);
    nh.getParam("/max_data", maxData);
    transferSub = nh.subscribe("/vehicles/" + vehicleName + "/transferring", 1, &DataCapacityModule::transferCallback, this);
    baseSub = nh.subscribe("/vehicles/" + vehicleName + "/atBase", 1, &DataCapacityModule::baseCallback, this);
	pub = nh.advertise<std_msgs::Float64>("/vehicle/" + vehicleName + "dataCapacity", 1000);
}

void DataCapacityModule::baseCallback(const std_msgs::Bool::ConstPtr& msg)
{
    inBaseRange = msg->data;
}

void DataCapacityModule::transferCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if(inBaseRange) {
    	sentData += transferRate;
    }
}


void DataCapacityModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	dataCapacity += sentData;
	if(dataCapacity > maxData) {
		dataCapacity = maxData;
	}
	sentData = 0.0;

	std_msgs::Float64 capacity;
	capacity.data = dataCapacity;
	pub.publish(capacity);
}
