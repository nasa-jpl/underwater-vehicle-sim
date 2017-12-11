#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "vehicles/Module.h"

/**
 * Class used to represent a vehicle in the simulation
 */
class Vehicle
{
public:

	Vehicle();


	void simCycle();
private:
	
	std::vector<Module> modules;
};

#endif