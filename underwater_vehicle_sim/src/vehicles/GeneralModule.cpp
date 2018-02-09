#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/DataRecorderModule.h"
#include "vehicles/PowerCapacityModule.h"
#include "vehicles/DataCapacityModule.h"
#include "vehicles/BaseStationModule.h"

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

	if(moduleType == "DataRecorder")
	{
		std::unique_ptr<GeneralModule> returnPtr(new DataRecorderModule(moduleName, parentNH, vehicleName));
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
	if(moduleType == "BaseStation")
	{
		std::unique_ptr<GeneralModule> returnPtr(new BaseStationModule(moduleName, parentNH, vehicleName));
		return returnPtr;
	}

	return NULL;
}

void GeneralModule::updateAtRate(std::string name, const ros::Time& lastTime, const tf::Vector3& position, 
							double& powerCapacity, double &dataCapacity)
{
	ros::Duration rate(1 / hertz);

	if(!useHertz || ros::Time::now() - lastUpdate >= rate)
	{
		lastUpdate = ros::Time::now();
		update(name, lastTime, position, powerCapacity, dataCapacity);
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
