#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "std_msgs/Bool.h"
#include "vehicles/BaseStationModule.h"

BaseStationModule::BaseStationModule(std::string name, ros::NodeHandle& parentNH, std:: string vehicleName) :
	GeneralModule(name, "BaseStation", parentNH, vehicleName)
{
	pub = nh.advertise<std_msgs::Bool>("/vehicles/" + vehicleName + "atBase", 1000);
	nh.getParam("base_x", base_x);
	nh.getParam("base_y", base_y);
	nh.getParam("base_z", base_z);
}

double BaseStationModule::distanceToBase(const tf::Vector3& position)
{
	double dist = (base_x - position.getX()) * (base_x - position.getX());
	dist = dist + (base_y - position.getY()) * (base_y - position.getY());
	dist = dist + (base_z - position.getZ()) * (base_z - position.getZ());
	return sqrt(dist);
}

void BaseStationModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	std_msgs::Bool base_msg;
	double dist = distanceToBase(position);
	base_msg.data = false;
	//Placeholder distance
	if (dist < 100.0)
	{
		base_msg.data = true;
	}	
	pub.publish(base_msg);
}
