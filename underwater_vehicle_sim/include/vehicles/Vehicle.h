#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "ros/ros.h"
#include "vehicles/Module.h"

/**
 * Class used to represent a vehicle in the simulation
 */
class Vehicle
{
public:

	Vehicle(std::string name, float startX, float startY, float startZ);


	void simCycle();
private:
	
	std::vector<Module> modules;
	std::string name;
};

#endif