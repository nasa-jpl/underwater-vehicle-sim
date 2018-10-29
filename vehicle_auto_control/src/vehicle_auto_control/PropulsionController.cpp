
#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/VehicleController.h"

#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "tf/transform_broadcaster.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

PropulsionController::PropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName) :
	controlNode(controlNode), 
	vehicleNode(vehicleNode),
	vehicleName(vehicleName)
{}

std::unique_ptr<PropulsionController> PropulsionController::makePropulsionController(std::string vehicleName, 
																					 underwater_vehicle_msgs::GetVehicleInfo info,
																					 ros::NodeHandle& parentNH)
{
	if(info.response.propModuleType == "FourDOFPropulsion")
	{
		std::string dataModuleName;

		for(unsigned int i = 0; i < info.response.moduleNames.size(); i++)
		{
			if(info.response.moduleTypes[i] == "DataBroadcaster")
			{
				dataModuleName = info.response.moduleNames[i];
			}
		}

		std::unique_ptr<PropulsionController> returnPtr(new FourDOFPropulsionController(ros::NodeHandle(parentNH, "vehicle_controller/" + vehicleName),
																						ros::NodeHandle(parentNH, "vehicles/" + vehicleName),
																						info.response.propModuleName,
																						dataModuleName,
																						vehicleName));
		return returnPtr;
	}

	return NULL;
}