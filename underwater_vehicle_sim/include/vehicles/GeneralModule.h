#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

class GeneralModule
{
public:
	GeneralModule(std::string name, ros::NodeHandle parentNH);

	virtual ~GeneralModule(){}

	virtual void update()=0;

private:
	ros::NodeHandle nh;
};


#endif