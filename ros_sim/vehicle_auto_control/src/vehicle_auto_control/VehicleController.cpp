#include "vehicle_auto_control/VehicleController.h"

#include "vehicle_auto_control/PropulsionController.h"

VehicleController::VehicleController()
{
	infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
	infoClient.waitForExistence();

	underwater_vehicle_msgs::GetVehicleInfo info;
	infoClient.call(info);
	VehicleInfo vehicleInfo(info);

	if(vehicleInfo.getPropModuleType() != "")
	{
		
		std::unique_ptr<PropulsionController> controller(new PropulsionController(vehicleInfo));
		propControllers.push_back(std::move(controller));
	}
}

void VehicleController::update(void)
{
	for(unsigned int i = 0; i < propControllers.size(); i++)
	{
		propControllers[i]->update();
	}
}