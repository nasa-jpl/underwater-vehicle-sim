#include <memory>

#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/PropulsionModule.h"
#include "vehicles/FourDOFPropulsion.h"

PropulsionModule::PropulsionModule(std::string name, ros::NodeHandle& parentNH) :
	name(name),
	nh(ros::NodeHandle(parentNH, name))
{

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