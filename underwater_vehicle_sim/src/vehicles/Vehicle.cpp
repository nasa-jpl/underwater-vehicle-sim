#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

#include "vehicles/Vehicle.h"
#include "vehicles/Module.h"


Vehicle::Vehicle(std::string name, float startX, float startY, float startZ) :
	name(name) 
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(startX, startY, startZ) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, 0);
  	transform.setRotation(q);
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}

void Vehicle::simCycle() 
{
	for(Module& module : modules)
	{
		module.simCycle();
	}
}