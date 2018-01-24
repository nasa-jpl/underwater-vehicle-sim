#ifndef POWER_CAPACTIY_MODULE_H
#define POWER_CAPACITY_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

class PowerCapacityModule : public GeneralModule
{

public:
	PowerCapacityModule(std::string name, ros::NodeHandle& parentNH);
	~PowerCapacityModule() {}

	void update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapactiy, double& dataCapacity);


	
private:
	/**
	*Callback for the velocity message which is used to control this module
	*@param vel Twist message used to control this module
	*/
	void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

	ros::Publisher pub;
};


#endif
