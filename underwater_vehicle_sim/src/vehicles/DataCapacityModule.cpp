#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#include "std_msgs/Float64.h"
#include "vehicles/DataCapacityModule.h"

#define SECONDS_IN_DAY 86400

DataCapacityModule::DataCapacityModule(std::string name, ros::NodeHandle& parentNH) :
	GeneralModule(name, parentNH)
{
	pub = nh.advertise<std_msgs::Float64>("/vehicle/dataCapacity", 1000);
}

void DataCapacityModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	dataCapacity = dataCapacity - 0.1;
	std_msgs::Float64 capacity;
	capacity.data = dataCapacity;
	pub.publish(capacity);
}
