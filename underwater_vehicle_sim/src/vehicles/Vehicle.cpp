#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "vehicles/FourDOFPropulsion.h"

Vehicle::Vehicle(std::string name, ros::NodeHandle& parentNH) :
	name(name),
	nh(ros::NodeHandle(parentNH, name))
{
	initalizeVehicleFrame();

	initalizePropulsionModule();
	initalizeGeneralModules();
}

Vehicle::Vehicle(Vehicle&& other)
	: propulsionModule(std::move(other.propulsionModule)), 
      modules(std::move(other.modules)),
      name(std::move(other.name)),
      nh(std::move(other.nh))
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
	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(startX, startY, startZ));
  	tf::Quaternion q;
  	q.setRPY(0, 0, 0);
  	transform.setRotation(q);
  	broadcastTransform(transform);
}

void Vehicle::initalizePropulsionModule()
{
	std::string propModuleName;

	//get the name of the propulsion module and create the needed 
	nh.getParam("propModuleName", propModuleName);
	propulsionModule = PropulsionModule::makePropulsionModule(propModuleName, nh);
}

void Vehicle::initalizeGeneralModules()
{

}

void Vehicle::update()
{
	//move the frame using the propulsion module and broadcast it
	tf::Transform movedTransform = propulsionModule->move(getVehicleFrame());
	broadcastTransform(movedTransform);

	//update all modules
	for(GeneralModule& module : modules)
	{
		module.update();
	}
}

tf::StampedTransform Vehicle::getVehicleFrame()
{
	tf::StampedTransform transform;
	try
	{
  		transformListener.lookupTransform(name, "/world",  
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
    	ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
}

void Vehicle::broadcastTransform(tf::Transform transform)
{
	static tf::TransformBroadcaster br;
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}
