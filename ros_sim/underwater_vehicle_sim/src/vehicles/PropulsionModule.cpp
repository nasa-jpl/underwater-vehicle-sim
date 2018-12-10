#include <memory>

#include "ros/ros.h"

#include "vehicles/Vehicle.h"
#include "vehicles/VehicleState.h"
#include "vehicles/PropulsionModule.h"
#include "vehicles/FourDOFPropulsion.h"

PropulsionModule::PropulsionModule(std::string name, std::string type, VehicleState& vehicleState, ros::NodeHandle& parentNH) :
	name(name),
	type(type),
	nh(ros::NodeHandle(parentNH, name)),
	vehicleState(vehicleState)
{}

std::unique_ptr<PropulsionModule> PropulsionModule::makePropulsionModule(std::string moduleName, VehicleState& vehicleState, ros::NodeHandle& parentNH)
{
	std::string moduleType;
	parentNH.getParam(moduleName + "/type", moduleType);

	if(moduleType == "FourDOFPropulsion")
	{
		std::unique_ptr<PropulsionModule> returnPtr(new FourDOFPropulsion(moduleName, vehicleState, parentNH));
		return returnPtr;
	}

	return NULL;
}

std::string& PropulsionModule::getName()
{
	return name;
}

std::string& PropulsionModule::getType()
{
	return type;
}
