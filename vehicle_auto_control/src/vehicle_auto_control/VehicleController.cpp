#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/PropulsionController.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

VehicleController::VehicleController(ros::NodeHandle& parentNH, float loopHertz) :
nh(parentNH),
loopHertz(loopHertz)
{
	//Create the vehicle objects
	std::vector<std::string> vehicleNames;
	nh.getParam("vehicles/names", vehicleNames);
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();
	std::string propModuleName;

	for(std::string& name : vehicleNames)
	{
		underwater_vehicle_sim::GetVehicleInfo info;
		info.request.name = name;
		infoClient.call(info);

		if(info.response.propModuleName != "")
		{
			propControllers.push_back(PropulsionController::makePropulsionController(name, info.response.propModuleName, info.response.propModuleType, nh, loopHertz));
		}
	}
}