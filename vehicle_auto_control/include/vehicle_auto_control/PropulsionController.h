#ifndef PROPULSION_CONTROLLER_H
#define PROPULSION_CONTROLLER_H

#include "ros/ros.h"

#include <memory>
#include <vector>

#include "tf/transform_broadcaster.h"

class PropulsionController
{

public:
	PropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName);
	virtual ~PropulsionController() {}

	
	virtual void update()=0;

	static std::unique_ptr<PropulsionController> makePropulsionController(std::string vehicleName, std::string moduleName, std::string moduleType, ros::NodeHandle& parentNH);	

protected:
	ros::NodeHandle controlNode;
	ros::NodeHandle vehicleNode;
	std::string vehicleName;
};

#endif