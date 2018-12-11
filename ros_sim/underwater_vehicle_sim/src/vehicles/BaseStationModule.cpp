#include "ros/ros.h"
#include "tf2/LinearMath/Vector3.h"

#include "std_msgs/Bool.h"
#include "vehicles/BaseStationModule.h"

BaseStationModule::BaseStationModule(std::string name, ros::NodeHandle& parentNH, std:: string vehicleName) :
	GeneralModule(name, "BaseStation", parentNH, vehicleName)
{
	pub = nh.advertise<std_msgs::Bool>("vehicles/" + vehicleName + "/atBase", 1000);
	nh.getParam("base_x", base_x);
	nh.getParam("base_y", base_y);
	nh.getParam("base_z", base_z);
    nh.getParam("base_range", base_range);
}

double BaseStationModule::distanceToBase(const tf2::Vector3& position)
{
	double dist = (base_x - position.getX()) * (base_x - position.getX());
	dist = dist + (base_y - position.getY()) * (base_y - position.getY());
	dist = dist + (base_z - position.getZ()) * (base_z - position.getZ());
	return sqrt(dist);
}

void BaseStationModule::update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState) 
{
	tf2::Vector3 position = vehicleState.getPosition();

	std_msgs::Bool base_msg;
	double dist = distanceToBase(position);
	base_msg.data = false;
	if (dist < base_range)
	{
		base_msg.data = true;
	}	
	pub.publish(base_msg);
}
