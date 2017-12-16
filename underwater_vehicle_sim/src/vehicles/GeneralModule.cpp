#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"

GeneralModule::GeneralModule(std::string name, ros::NodeHandle parentNH) :
	nh(ros::NodeHandle(parentNH, name))

{}