#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"

#include "vehicles/DataBroadcasterModule.h"
#include "vehicles/IMUModule.h"

GeneralModule::GeneralModule(std::string name, std::string type) :
	nh(name),
	name(name),
	type(type)
{
	ros::NodeHandle nhPriv("~/" + name);
	if(nhPriv.hasParam("hertz"))
	{
		useHertz = true;
		nhPriv.getParam("hertz", hertz);
	}
	else
	{
		useHertz = false;
		hertz = 1;
	}
}

std::unique_ptr<GeneralModule> GeneralModule::makeGeneralModule(std::string moduleName)
{
	ros::NodeHandle nh(moduleName);
	ros::NodeHandle nhPriv("~/" + moduleName);
	std::string moduleType;	
	nhPriv.getParam("type", moduleType);

	if(moduleType == "DataBroadcaster")
	{
		std::unique_ptr<GeneralModule> returnPtr(new DataBroadcasterModule(moduleName));
		return returnPtr;
	}
	if(moduleType == "IMU")
	{
		std::unique_ptr<GeneralModule> returnPtr(new IMUModule(moduleName));
		return returnPtr;
	}

	return NULL;
}

void GeneralModule::updateAtRate(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& data)
{
	ros::Duration rate(1 / hertz);

	if(!useHertz || ros::Time::now() - lastUpdate >= rate)
	{
		lastUpdate = ros::Time::now();
		update(lastTime, vehicleState, data);
	}
}

std::string& GeneralModule::getName()
{
	return name;
}

std::string& GeneralModule::getType()
{
	return type;
}
