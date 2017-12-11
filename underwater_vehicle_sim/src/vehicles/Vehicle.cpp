#include "vehicles/Vehicle.h"
#include "vehicles/Module.h"
Vehicle::Vehicle() {}

void Vehicle::simCycle() 
{
	for(Module& module : modules)
	{
		module.simCycle();
	}
}