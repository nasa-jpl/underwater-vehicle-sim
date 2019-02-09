#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <memory>

#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "model_interface/ModelInterface.h"
#include "model_interface/ModelData.h"
#include "model_server/GetModelData.h"

/**
 * Class used to represent a vehicle in the simulation
 * Vehicle front is the position x-axis in the vehicle frame
 * Vehicle rotation follows the standard right-hand rule with 0 degrees on the positive x-axis in the world frame
 */
class Vehicle
{
public:
	Vehicle();
	Vehicle(std::unique_ptr<ModelInterface> model);
	Vehicle(Vehicle&& other);
	
	void update();

	std::string getName();
	void getInfo(underwater_vehicle_msgs::GetVehicleInfo::Response &res);
	
private:

	/**
	* Broadcast this vehicles pose relative to the world frame using tf transforms
	* @param transform Transform to broadcast
	*/
	void broadcastTransform();

	/**
	* Initalizes the general modules. Starting the callback timers
	*/
	void initalizeGeneralModules();

	/**
	* Initalizes the propulsion module. Starting the callback timer
	*/
	void initalizePropulsionModule();

	/**
	* Initalizes the vehicle frame using tf
	*/
	void initalizeVehicleFrame();

	/**
	* Get data from the model at the current vehicle state and last transform time
	*/
	ModelData getModelData();

	/**
	 * Callback to call module update. This is called by a timer callback
	 * @param moduleIndex The index of the module to call update on
	 */
	void moduleTimerCallback(unsigned int moduleIndex);

	/**
	 * Callback to call propulsion module update. This is called by a timer callback.
	 */
	void propModuleTimerCallback();
private:

    /**
	 * ros node handle for this vehicle
	 */
	ros::NodeHandle nh;

	/**
	 * Holds entire current vehicle state
	 */
	VehicleState vehicleState;

	/**
	* Time the last transform was sent
	*/
	ros::Time lastTransformTime;

	/**
	* Model data from the last transform location and time
	*/
	ModelData dataAtLastTransform;

	/**
	 * Module that handles the vehicles propulsion system
	 */
	std::unique_ptr<PropulsionModule> propulsionModule;

	/**
	 * Modules to handle all other vehicle tasks
	 */
	std::vector<std::unique_ptr<GeneralModule>> modules;

	/**
	 * Timer to handle update
	 */
	std::vector<ros::Timer> moduleTimers;

	/**
	 * Time to handle updates of the propulsion module
	 */
	ros::Timer propTimer;

	/**
	* Tracks power remaining for this vehicle
	*/
	double powerCapacity; 
	
	/**
	* Tracks data storage remaining for this vehicle
	*/
	double dataCapacity;

	/**
    * Model client used to get the model data at the vehicle location
    */
    ros::ServiceClient modelClient;

	/**
	* Dictates if currents evect the vehicle
	*/
	bool evectByCurrents;

	/**
	* Starting X position of the vehicle
	*/
	double startX;

	/**
	* Starting Y position of the vehicle
	*/
	double startY;

	/**
	* Starting Z position of the vehicle
	*/
	double startZ;

	/**
	* Local model used if speed in important
	*/
	std::unique_ptr<ModelInterface> model;

};

#endif