#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

UnderwaterVehicleSim::UnderwaterVehicleSim() {}

void UnderwaterVehicleSim::update() 
{
	for(Vehicle& vehicle : vehicles)
	{
		vehicle.update();
	}
}