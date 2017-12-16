#ifndef PROPULSION_MODULE_H
#define PROPULSION_MODULE_H

#include "ros/ros.h"

class PropulsionModule
{

public:
	PropulsionModule(std::string name, ros::NodeHandle& parentNH);
	virtual ~PropulsionModule() {}

	/**
	* Calculates the new frame of the vehicle from the old one
	* @param currentLocaion The old vehicle frame relative to the world frame
	* @return The new vehicle frame relative to the world frame
	*/
	virtual tf::Transform move(tf::StampedTransform currentLocation)=0;

	/**
	* Creates a propulsion module using the parameters from the parameter server
	* @oaram moduleName The name of the module which is used for parameters
	* @param parentNH The parent node handle for this ros node
	* @return A pointer to the newly created module
	*/
	static std::unique_ptr<PropulsionModule> makePropulsionModule(std::string moduleName, ros::NodeHandle& parentNH);
protected:
	ros::NodeHandle nh;
};

#endif