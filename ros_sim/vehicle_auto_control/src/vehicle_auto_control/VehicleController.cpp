#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/PropulsionController.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

VehicleController::VehicleController(ros::NodeHandle& parentNH) :
nh(parentNH)
{
	//Create the vehicle objects
	std::vector<std::string> vehicleNames;
	nh.getParam("vehicles/names", vehicleNames);
	infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();
	std::string propModuleName;

	for(std::string& name : vehicleNames)
	{
		underwater_vehicle_msgs::GetVehicleInfo info;
		info.request.name = name;
		infoClient.call(info);

		if(info.response.propModuleName != "")
		{
			VehicleInfo vehicleInfo(info);
			PropulsionController controller(nh, vehicleInfo);
		}
	}
}

void VehicleController::update(void)
{
	for(unsigned int i = 0; i < propControllers.size(); i++)
	{
		propControllers[i].update();
	}
}