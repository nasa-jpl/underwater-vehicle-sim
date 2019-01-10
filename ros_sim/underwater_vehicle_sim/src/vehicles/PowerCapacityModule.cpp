#include <iostream>
#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "vehicles/PowerCapacityModule.h"

PowerCapacityModule::PowerCapacityModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "PowerCapacity", parentNH, vehicleName)
{
    savedCharge = 0.0;
    inBaseRange = false;
    nh.getParam("charge_rate", chargeRate);
    nh.getParam("max_charge", maxCharge);
    chargingSub = nh.subscribe("underwater_vehicle_sim/vehicles/" + vehicleName + "/charging", 1, &PowerCapacityModule::chargingCallback, this);
    baseSub = nh.subscribe("underwater_vehicle_sim/vehicles/" + vehicleName + "/atBase", 1, &PowerCapacityModule::baseCallback, this);
	pub = nh.advertise<std_msgs::Float64>("underwater_vehicle_sim/vehicles/" + vehicleName + "/power", 1000);
}

void PowerCapacityModule::baseCallback(const std_msgs::Bool::ConstPtr& msg)
{
    inBaseRange = msg->data;
}

void PowerCapacityModule::chargingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    if(inBaseRange) {
        savedCharge += chargeRate;
    }
}

void PowerCapacityModule::update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	std_msgs::Float64 power_msg;
	power_msg.data = vehicleState.getPowerCapacity();
	pub.publish(power_msg);
}
