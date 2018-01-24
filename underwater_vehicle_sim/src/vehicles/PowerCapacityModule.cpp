#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "std_msgs/Float64.h"
#include "vehicles/PowerCapacityModule.h"

PowerCapacityModule::PowerCapacityModule(std::string name, ros::NodeHandle& parentNH) :
	GeneralModule(name, parentNH)
{
	nh.getParam("start_power", capacity);
	pub = nh.advertise<std_msgs::Float64>("/vehicles/power", 1000);
}

void PowerCapacityModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position) 
{
	capacity = capacity - 0.1;
	std_msgs::Float64 msg;
	msg.data = capacity;
	pub.publish(msg);
}
