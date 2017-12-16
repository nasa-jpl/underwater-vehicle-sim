#ifndef UNDERWATER_VEHICLE_SIM_H
#define UNDERWATER_VEHICLE_SIM_H

#include "vehicles/Vehicle.h"

/**
 * Class used to simulate underwater vehicles using a model
 */
class UnderwaterVehicleSim
{
public:

	UnderwaterVehicleSim(ros::NodeHandle& parentNH);

	/**
	* The update method for the simulation that runs once per sim cycle
	*/
	void update();
private:
	/**
	* All the vehicles in the simulation
	*/
	std::vector<Vehicle> vehicles;

	ros::NodeHandle& nh;
};

#endif