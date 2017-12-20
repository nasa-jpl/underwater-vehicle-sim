#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "vehicles/FourDOFPropulsion.h"

Vehicle::Vehicle(std::string name, ros::NodeHandle& parentNH) :
	name(name),
	nh(ros::NodeHandle(parentNH, "vehicles/" + name))
{
	initalizeVehicleFrame();
  	
	initalizePropulsionModule();
	initalizeGeneralModules();
}

Vehicle::Vehicle(Vehicle&& other)
	: propulsionModule(std::move(other.propulsionModule)), 
      modules(std::move(other.modules)),
      name(std::move(other.name)),
      nh(std::move(other.nh)),
      position(std::move(other.position)),
      rotation(std::move(other.rotation)),
      lastTransformTime(std::move(other.lastTransformTime))
{}

void Vehicle::initalizeVehicleFrame()
{
	//Get the parameters for the starting location of the vehicle
	float startX = 0;
	float startY = 0;
	float startZ = 0;

	nh.getParam("start_x", startX);
	nh.getParam("start_y", startY);
	nh.getParam("start_z", startZ);

	//broadcast the inital frame for this vehicle
  	
  	rotation.setRPY(0, 0, 0);

  	position.setX(startX);
  	position.setY(startY);
  	position.setZ(startZ);
  	broadcastTransform();
  	
}

void Vehicle::initalizePropulsionModule()
{
	std::string propModuleName;

	//get the name of the propulsion module and create the needed 
	if(nh.hasParam("propModuleName"))
	{
		nh.getParam("propModuleName", propModuleName);
		propulsionModule = PropulsionModule::makePropulsionModule(propModuleName, nh);
	}
}

void Vehicle::initalizeGeneralModules()
{

}

void Vehicle::update()
{
	//move the frame using the propulsion module and broadcast it
	if(propulsionModule)
	{
		propulsionModule->move(lastTransformTime, rotation, position);
	}

	broadcastTransform();

	//update all modules
	for(GeneralModule& module : modules)
	{
		module.update();
	}
}

void Vehicle::broadcastTransform()
{
	static tf::TransformBroadcaster br;
	lastTransformTime = ros::Time::now();
  	br.sendTransform(tf::StampedTransform(tf::Transform(rotation, position), lastTransformTime, "world", name));
}

std::string Vehicle::getName()
{
	return name;
}