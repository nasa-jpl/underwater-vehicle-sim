#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <memory>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "model_server/GetModelData.h"

/**
 * Class used to represent a vehicle in the simulation
 * Vehicle front is the position x-axis in the vehicle frame
 * Vehicle rotation follows the standard right-hand rule with 0 degrees on the positive x-axis in the world frame
 */
class Vehicle
{
public:
	Vehicle(std::string name, ros::NodeHandle& parentNH);
	Vehicle(Vehicle&& other);
	
	void update();

	std::string getName();
	void getInfo(underwater_vehicle_sim::GetVehicleInfo::Response &res);
	
private:

	/**
	* Broadcast this vehicles pose relative to the world frame using tf transforms
	* @param transform Transform to broadcast
	*/
	void broadcastTransform();
	
	/**
	* Initalizes the propulsion module the propulsion module
	*/
	void initalizePropulsionModule();

	/**
	* Initalizes the general modules
	*/
	void initalizeGeneralModules();

	/**
	* Initalizes the vehicle frame using tf
	*/
	void initalizeVehicleFrame();

	

private:
	tf::TransformListener transformListener;

	/**
	* Current vehicle rotation expressed as a tf quaternion
	*/
	tf::Quaternion rotation;

	/**
	* Current vehicle position espressed as a tf vector
	*/
	tf::Vector3 position;

	/**
	* Time the last transform was sent
	*/
	ros::Time lastTransformTime;

	/**
	 * Module that handles the vehicles propulsion system
	 */
	std::unique_ptr<PropulsionModule> propulsionModule;

	/**
	 * Modules to handle all other vehicle tasks
	 */
	std::vector<std::unique_ptr<GeneralModule>> modules;

	/**
	 * name of the vehicle
	 */
	std::string name;

	/**
	 * ros node handle for this vehicle
	 */
	ros::NodeHandle nh;
};

#endif