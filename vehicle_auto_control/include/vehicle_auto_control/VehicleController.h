#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "ros/ros.h"

#include <memory>
#include <vector>

#include "vehicle_auto_control/PropulsionController.h"

class VehicleController
{
public:
	VehicleController(ros::NodeHandle& parentNH);

	void update();

private:
	ros::NodeHandle& nh;
	std::vector<std::unique_ptr<PropulsionController>> propControllers;
};

#endif