#include "vehicle_auto_control/VehicleController.h"

#include "vehicle_auto_control/PropulsionController.h"

VehicleController::VehicleController()
{
	std::vector<std::string> vehicleNames;
	nh.getParam("underwater_vehicle_sim/vehicles/names", vehicleNames);


	infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
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
			std::unique_ptr<PropulsionController> controller(new PropulsionController(vehicleInfo));
		 	propControllers.push_back(std::move(controller));
		}
	}
}

void VehicleController::update(void)
{
	for(unsigned int i = 0; i < propControllers.size(); i++)
	{
		propControllers[i]->update();
	}
}