#ifndef SIX_DOF_PROPULSION_H
#define SIX_DOF_PROPULSION_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include "vehicles/PropulsionModule.h"
#include "vehicles/LinearPiecewise.h"

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

	/**
	* Callback for the message controlling the forward thruster
	*/
	void forwardThrusterCallback(const std_msgs::Float64::ConstPtr& val);

	/**
	* Callback for the message controlling the lateral thruster
	*/
	void lateralThrusterCallback(const std_msgs::Float64::ConstPtr& val);

	/**
	* Callback for the message controlling the vertical thruster
	*/
	void verticalThrusterCallback(const std_msgs::Float64::ConstPtr& val);

	/**
	* Callback for the message controlling the rudder
	*/
	void rudderCallback(const std_msgs::Float64::ConstPtr& val);


private:

	ros::Subscriber forwardThrusterSub;
	ros::Subscriber lateralThrusterSub;
	ros::Subscriber verticalThrusterSub;
	ros::Subscriber rudderSub;

	LinearPiecewise forwardThrusterFunc;
	LinearPiecewise lateralThrusterFunc;
	LinearPiecewise verticalThrusterFunc;
	LinearPiecewise rudderFunc;
};


#endif
