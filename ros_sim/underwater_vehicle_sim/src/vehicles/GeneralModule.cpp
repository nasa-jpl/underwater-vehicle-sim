#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"

#include "vehicles/DataBroadcasterModule.h"
#include "vehicles/PowerCapacityModule.h"
#include "vehicles/DataCapacityModule.h"
#include "vehicles/IMUModule.h"

GeneralModule::GeneralModule(std::string name, std::string type, 
                ros::NodeHandle parentNH, std::string vehicleName) :
	nh(ros::NodeHandle(parentNH, name)),
	name(name),
	type(type),
    vehicleName(vehicleName)

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

std::unique_ptr<GeneralModule> GeneralModule::makeGeneralModule(std::string moduleName, 
                            ros::NodeHandle& parentNH, std::string vehicleName)
{
	std::string moduleType;
	parentNH.getParam(moduleName + "/type", moduleType);

	if(moduleType == "DataBroadcaster")
	{
		std::unique_ptr<GeneralModule> returnPtr(new DataBroadcasterModule(moduleName, parentNH, vehicleName));
		return returnPtr;
	}
	if(moduleType == "PowerCapacity")
	{
		std::unique_ptr<GeneralModule> returnPtr(new PowerCapacityModule(moduleName, parentNH, vehicleName));
		return returnPtr;
	}
	if(moduleType == "DataCapacity")
	{
		std::unique_ptr<GeneralModule> returnPtr(new DataCapacityModule(moduleName, parentNH, vehicleName));
		return returnPtr;
	}
	if(moduleType == "IMU")
	{
		std::unique_ptr<GeneralModule> returnPtr(new IMUModule(moduleName, parentNH, vehicleName));
		return returnPtr;
	}

	return NULL;
}

void GeneralModule::updateAtRate(std::string name, const ros::Time& lastTime, VehicleState& vehicleState)
{
	ros::Duration rate(1 / hertz);

	if(!useHertz || ros::Time::now() - lastUpdate >= rate)
	{
		lastUpdate = ros::Time::now();
		update(name, lastTime, vehicleState);
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
