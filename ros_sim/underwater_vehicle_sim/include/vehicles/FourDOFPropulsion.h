#ifndef SIX_DOF_PROPULSION_H
#define SIX_DOF_PROPULSION_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "vehicles/PropulsionModule.h"

/**
*Propulson module which provides the vehicle with 4 degrees of freedom
* Heave/Sway/Surge/Yaw (Linear X/Y/Z, Rotational Z)
*/
class FourDOFPropulsion : public PropulsionModule
{

public:
	FourDOFPropulsion(std::string name, VehicleState& vehicleState, ros::NodeHandle& parentNH);
	~FourDOFPropulsion() {}
	
private:
	/**
	*Callback for the velocity message which is used to control this module
	*@param vel Twist message used to control this module
	*/
	void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

private:
	/**
	*Max linear velocity for x,y
	*/
	double maxLinVelocity;

	/**
	*Max linear velocity for z
	*/
	double maxVertVelocity;
	
	/**
	*Max rotational velocity for all DOF
	*/
	double maxRotVelocity;

	ros::Subscriber commandVelocitySub;
};


#endif
