#ifndef PROPULSION_CONTROLLER_H
#define PROPULSION_CONTROLLER_H

#include "ros/ros.h"

#include <memory>
#include <vector>

#include "tf/transform_broadcaster.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

class PropulsionController
{

public:
	PropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName);
	virtual ~PropulsionController() {}

	
	virtual void update(void)=0;

	static std::unique_ptr<PropulsionController> makePropulsionController(std::string vehicleName, 
																		  underwater_vehicle_sim::GetVehicleInfo info,
																		  ros::NodeHandle& parentNH);	

protected:
	ros::NodeHandle controlNode;
	ros::NodeHandle vehicleNode;
	std::string vehicleName;
};

#endif