#ifndef UNDERWATER_VEHICLE_SIM_H
#define UNDERWATER_VEHICLE_SIM_H

#include "vehicles/Vehicle.h"

/**
 * Class used to simulate underwater vehicles using a model
 */
class UnderwaterVehicleSim
{
public:

	UnderwaterVehicleSim();

	void update();
private:
	std::vector<Vehicle> vehicles;
};

#endif