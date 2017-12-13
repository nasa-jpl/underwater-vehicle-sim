#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "vehicles/SixDOFPropulsion.h"

Vehicle::Vehicle(std::string name, float startX, float startY, float startZ) :
	name(name)
{
	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(startX, startY, startZ) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, 0);
  	transform.setRotation(q);
  	broadcastTransform(transform);
}

void Vehicle::update()
{

	tf::StampedTransform movedTransform = propulsionModule->move(getVehicleFrame());
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