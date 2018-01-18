#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/DataRecorderModule.h"
GeneralModule::GeneralModule(std::string name, ros::NodeHandle parentNH) :
	nh(ros::NodeHandle(parentNH, name))

{}

std::unique_ptr<GeneralModule> GeneralModule::makeGeneralModule(std::string moduleName, ros::NodeHandle& parentNH)
{
	std::string moduleType;
	parentNH.getParam(moduleName + "/type", moduleType);

	if(moduleType == "DataRecorder")
	{
		std::unique_ptr<GeneralModule> returnPtr(new DataRecorderModule(moduleName, parentNH));
		return returnPtr;
	}

	return NULL;
}