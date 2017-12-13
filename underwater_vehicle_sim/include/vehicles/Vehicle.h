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
	Vehicle(std::string name, float startX, float startY, float startZ);


	void update();
private:

	void broadcastTransform(tf::Transform transform);
	tf::StampedTransform getVehicleFrame();
private:
	tf::TransformListener transformListener;

	std::unique_ptr<PropulsionModule> propulsionModule;
	std::vector<GeneralModule> modules;
	std::string name;
};

#endif