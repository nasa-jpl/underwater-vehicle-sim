#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

UnderwaterVehicleSim::UnderwaterVehicleSim() {}

void UnderwaterVehicleSim::update() 
{
	//Update all vehicles in this simulation
	for(Vehicle& vehicle : vehicles)
	{
		vehicle.update();
	}
}