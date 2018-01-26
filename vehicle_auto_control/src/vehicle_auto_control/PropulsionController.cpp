
#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/VehicleController.h"

#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "tf/transform_broadcaster.h"

PropulsionController::PropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName) :
	controlNode(controlNode), 
	vehicleNode(vehicleNode),
	vehicleName(vehicleName)
{}

std::unique_ptr<PropulsionController> PropulsionController::makePropulsionController(std::string vehicleName, std::string moduleName, std::string moduleType, ros::NodeHandle& parentNH)
{
	if(moduleType == "FourDOFPropulsion")
	{
		std::unique_ptr<PropulsionController> returnPtr(new FourDOFPropulsionController(ros::NodeHandle(parentNH, "vehicle_controller/" + vehicleName),
																						ros::NodeHandle(parentNH, "vehicles/" + vehicleName + "/" + moduleName),
																						vehicleName));
		return returnPtr;
	}

	return NULL;
}