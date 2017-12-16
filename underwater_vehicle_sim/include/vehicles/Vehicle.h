#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <memory>

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"


/**
 * Class used to represent a vehicle in the simulation
 */
class Vehicle
{
public:
	Vehicle(std::string name, ros::NodeHandle& parentNH);
	Vehicle(Vehicle&& other);
	
	void update();

	static void makeVehicle(std::string name, ros::NodeHandle parentNH);

private:

	/**
	* Broadcast this vehicles pose relative to the world frame using tf transforms
	* @param transform Transform to broadcast
	*/
	void broadcastTransform(tf::Transform transform);
	
	/**
	* Get this vehicles frame relative to the world frame using tf
	* @return The frame from this vehicle
	*/
	tf::StampedTransform getVehicleFrame();

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
	 * Module that handles the vehicles propulsion system
	 */
	std::unique_ptr<PropulsionModule> propulsionModule;

	/**
	 * Modules to handle all other vehicle tasks
	 */
	std::vector<GeneralModule> modules;

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