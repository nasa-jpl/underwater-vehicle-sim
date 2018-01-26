#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

UnderwaterVehicleSim::UnderwaterVehicleSim(ros::NodeHandle& parentNH) :
	nh(parentNH)
{
	//Create the vehicle objects
	std::vector<std::string> vehicleNames;
	nh.getParam("vehicles/names", vehicleNames);

	for(std::string& name : vehicleNames)
	{
		vehicles.emplace_back(name, nh);
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