#include <iostream>
#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "std_msgs/Float64.h"
#include "vehicles/PowerCapacityModule.h"

PowerCapacityModule::PowerCapacityModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "PowerCapacity", parentNH, vehicleName)
{
    savedCharge = 0.0;

    nh.getParam("charge_rate", chargeRate);
    chargingSub = nh.subscribe("/vehicles/" + vehicleName + "/charging", 1, &PowerCapacityModule::chargingCallback, this);
	pub = nh.advertise<std_msgs::Float64>("/vehicles/" + vehicleName + "/power", 1000);
}

void PowerCapacityModule::chargingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    savedCharge += chargeRate;
}

void PowerCapacityModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
    powerCapacity += savedCharge;

	std_msgs::Float64 power_msg;
	power_msg.data = powerCapacity;
	pub.publish(power_msg);
}
