#include <memory>

#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/PropulsionModule.h"
#include "vehicles/FourDOFPropulsion.h"

PropulsionModule::PropulsionModule(std::string name, ros::NodeHandle& parentNH) :
	name(name),
	nh(ros::NodeHandle(parentNH, name))
{
	if(nh.hasParam("hertz"))
	{
		useHertz = true;
		nh.getParam("hertz", hertz);
	}
	else
	{
		useHertz = false;
		hertz = 1;
	}
}

std::unique_ptr<PropulsionModule> PropulsionModule::makePropulsionModule(std::string moduleName, ros::NodeHandle& parentNH)
{
	std::string moduleType;
	parentNH.getParam(moduleName + "/type", moduleType);

	if(moduleType == "FourDOFPropulsion")
	{
		std::unique_ptr<PropulsionModule> returnPtr(new FourDOFPropulsion(moduleName, parentNH));
		return returnPtr;
	}

	return NULL;
}

void PropulsionModule::moveAtRate(ros::Time& lastTime, tf::Quaternion& rotation, tf::Vector3& position)
{
	ros::Duration rate(1 / hertz);

	if(!useHertz || ros::Time::now() - lastUpdate >= rate)
	{
		lastUpdate = ros::Time::now();
		move(lastTime, rotation, position);
	}
}