#include <memory>

#include "ros/ros.h"

#include "vehicles/Vehicle.h"
#include "vehicles/VehicleState.h"
#include "vehicles/PropulsionModule.h"
#include "vehicles/FourDOFPropulsion.h"

PropulsionModule::PropulsionModule(std::string type, VehicleState& vehicleState) :
	type(type),
	vehicleState(vehicleState)
{}

std::unique_ptr<PropulsionModule> PropulsionModule::makePropulsionModule(VehicleState& vehicleState)
{
	ros::NodeHandle nh;
	ros::NodeHandle nhPriv("~");
	std::string moduleType;
	nhPriv.getParam("propulsion_type", moduleType);

	if(moduleType == "FourDOFPropulsion")
	{
		std::unique_ptr<PropulsionModule> returnPtr(new FourDOFPropulsion(vehicleState));
		return returnPtr;
	}

	return NULL;
}

std::string& PropulsionModule::getType()
{
	return type;
}
