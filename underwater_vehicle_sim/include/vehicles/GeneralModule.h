#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

class GeneralModule
{
public:
	GeneralModule(std::string name, ros::NodeHandle parentNH);

	virtual ~GeneralModule(){}

	virtual void update(const ros::Time& lastTime, const tf::Vector3& position)=0;

protected:
	ros::NodeHandle nh;
};


#endif