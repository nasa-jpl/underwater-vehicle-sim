#ifndef SIX_DOF_PROPULSION_H
#define SIX_DOF_PROPULSION_H

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/PropulsionModule.h"

/**
*Propulson module which provides the vehicle with 4 degrees of freedom
* Heave/Sway/Surge/Yaw (Linear X/Y/Z, Rotational Z)
*/
class FourDOFPropulsion : public PropulsionModule
{

public:
	FourDOFPropulsion(std::string name, ros::NodeHandle& parentNH);
	~FourDOFPropulsion() {}

	/**
	* Calculates the new frame of the vehicle from the old one
	*/
	void move(ros::Time& lastTime, tf::Quaternion& rotation, tf::Vector3& position, 
					double& powerCapacity, double& dataCapacity);


	
private:
	/**
	*Callback for the velocity message which is used to control this module
	*@param vel Twist message used to control this module
	*/
	void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

private:
	/**
	*Max linear velocity for all DOF
	*/
	float maxLinVelocity;
	
	/**
	*Max rotational velocity for all DOF
	*/
	float maxRotVelocity;

	/**
	*Current linear velocity for this module
	*/
	tf::Vector3 linVelocity;

	/**
	*Current rotation velocity for this module
	*/
	tf::Vector3 rotVelocity;

	ros::Subscriber commandVelocitySub;

	ros::ServiceClient modelClient;
};


#endif
