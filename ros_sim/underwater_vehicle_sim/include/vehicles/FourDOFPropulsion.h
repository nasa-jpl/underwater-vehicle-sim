#ifndef FOUR_DOF_PROPULSION_H
#define FOUR_DOF_PROPULSION_H

#include <random>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include "vehicles/PropulsionModule.h"

#include "underwater_util/LinearPiecewise.h"

/**
*Propulson module which provides the vehicle with 4 degrees of freedom
* Heave/Sway/Surge/Yaw (Linear X/Y/Z, Rotational Z)
*/
class FourDOFPropulsion : public PropulsionModule
{

public:
	FourDOFPropulsion(VehicleState& vehicleState);
	~FourDOFPropulsion() {}
	
	void update() override;
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

	/**
	*Update the twist based on the recieved commands
	*/
	void updateTwist();

	/**
	*Publish sensors on thrust values and rudder values
	*/
	void publishSensors();

private:

	/*
	*Recieves commands for thrusters and rudders
	*/
	ros::Subscriber forwardThrusterSub;
	ros::Subscriber lateralThrusterSub;
	ros::Subscriber verticalThrusterSub;
	ros::Subscriber rudderSub;

	/*
	*Publishes sensors for actual thrust and rudder values
	*/
	ros::Publisher forwardThrusterPub;
	ros::Publisher lateralThrusterPub;
	ros::Publisher verticalThrusterPub;
	ros::Publisher rudderPub;

	/*
	*Functions defining thrust to velocity
	*/
	LinearPiecewise forwardThrusterFunc;
	LinearPiecewise lateralThrusterFunc;
	LinearPiecewise verticalThrusterFunc;
	LinearPiecewise rudderFunc;

	/*
	*Latest thrust commands
	*/
	double forwardThrust;
	double lateralThrust;
	double verticalThrust;
	double rudder;

	/*
	*Random distributions for sensor measurements
	*/
	std::default_random_engine generator;
	std::normal_distribution<double> thrustSensorDistribution;
	std::normal_distribution<double> rudderSensorDistribution;
	double thrustSensorRandomNoise;
	double rudderSensorRandomNoise;
};


#endif
