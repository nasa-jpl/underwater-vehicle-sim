#ifndef UNDERWATER_VEHICLE_SIM_H
#define UNDERWATER_VEHICLE_SIM_H

#include "vehicles/Vehicle.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"

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
	bool getVehicleInfo(underwater_vehicle_msgs::GetVehicleInfo::Request &req,
				  						  underwater_vehicle_msgs::GetVehicleInfo::Response &res);
private:
	/**
	* All the vehicles in the simulation
	*/
	std::vector<Vehicle> vehicles;

	ros::NodeHandle& nh;
	ros::ServiceServer service;
};

#endif