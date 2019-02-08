#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"

#include "vehicles/DataBroadcasterModule.h"
#include "vehicles/IMUModule.h"
#include "vehicles/USBLModule.h"

GeneralModule::GeneralModule(std::string name, std::string type) :
	nh(name),
	name(name),
	type(type)
{}

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
	if(moduleType == "USBL")
	{
		std::unique_ptr<GeneralModule> returnPtr(new USBLModule(moduleName));
		return returnPtr;
	}

	return NULL;
}

std::string& GeneralModule::getName()
{
	return name;
}

std::string& GeneralModule::getType()
{
	return type;
}
