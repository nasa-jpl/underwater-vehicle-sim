#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/PropulsionController.h"

VehicleController::VehicleController(ros::NodeHandle& parentNH) :
nh(parentNH)
{
	//Create the vehicle objects
	std::vector<std::string> vehicleNames;
	nh.getParam("vehicles/names", vehicleNames);

	std::string propModuleName;

	for(std::string& name : vehicleNames)
	{
		if(nh.hasParam("vehicles/" + name + "/propModuleName"))
		{
			nh.getParam("vehicles/" + name + "/propModuleName", propModuleName);
			propControllers.push_back(PropulsionController::makePropulsionController(name, propModuleName, nh));
		}
	}
}

void VehicleController::update()
{
	for(std::unique_ptr<PropulsionController>& controller : propControllers)
	{
		controller->update();
	}
}