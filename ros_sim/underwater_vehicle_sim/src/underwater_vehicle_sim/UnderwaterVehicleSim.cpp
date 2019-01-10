#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "vehicles/Vehicle.h"

UnderwaterVehicleSim::UnderwaterVehicleSim(ros::NodeHandle& parentNH) :
	nh(parentNH)
{
	service = nh.advertiseService("underwater_vehicle_sim/vehicles/get_info", &UnderwaterVehicleSim::getVehicleInfo, this);
	//Create the vehicle objects
	std::vector<std::string> vehicleNames;
	nh.getParam("underwater_vehicle_sim/vehicles/names", vehicleNames);

	bool evectByCurrents = nh.param("underwater_vehicle_sim/evect_by_currents", true);
	for(std::string& name : vehicleNames)
	{
		vehicles.emplace_back(name, nh, evectByCurrents);
	}
}

void UnderwaterVehicleSim::update()
{
	//Update all vehicles in this simulation
	for(Vehicle& vehicle : vehicles)
	{
		vehicle.update();
	}
}

bool UnderwaterVehicleSim::getVehicleInfo(underwater_vehicle_msgs::GetVehicleInfo::Request &req,
				  						  underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
	for(Vehicle& vehicle : vehicles)
	{
		if(vehicle.getName() == req.name)
		{
			vehicle.getInfo(res);
		}
	}
	return true;
}