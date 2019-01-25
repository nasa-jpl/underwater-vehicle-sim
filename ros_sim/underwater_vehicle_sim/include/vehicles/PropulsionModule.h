#ifndef PROPULSION_MODULE_H
#define PROPULSION_MODULE_H

#include "ros/ros.h"

#include "vehicles/VehicleState.h"

class PropulsionModule
{

public:
	/**
	* Constructor for propulsion modules
	* @param name Name of the propulsion module
	* @param type The type of propulsion module
	* @param vehicleState The current state of the vehicle. This is a reference so the propulsion module can modify it as needed.
	* @param parentNH NodeHandle for the vehicle
	*/
	PropulsionModule(std::string name, std::string type, VehicleState& vehicleState, ros::NodeHandle& parentNH);
	virtual ~PropulsionModule() {}

	/**
	* Creates a propulsion module using the parameters from the parameter server
	* @oaram moduleName The name of the module which is used for parameters
	* @param parentNH The parent node handle for this ros node
	* @return A pointer to the newly created module
	*/
	static std::unique_ptr<PropulsionModule> makePropulsionModule(std::string moduleName, VehicleState& vehicleState, ros::NodeHandle& parentNH);

	std::string& getName();
	std::string& getType();
protected:
	std::string name;
	std::string type;
	ros::NodeHandle nh;
	VehicleState& vehicleState;

	ros::Time lastUpdate;
};

#endif
