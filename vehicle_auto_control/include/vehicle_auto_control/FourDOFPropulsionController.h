#ifndef FOUR_DOF_PROPULSION_CONTROLLER_H
#define FOUR_DOF_PROPULSION_CONTROLLER_H

#include "tf/transform_listener.h"

#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/PointPathAction.h"

class FourDOFPropulsionController : public PropulsionController
{

public:
	FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName, float loopHertz);
	~FourDOFPropulsionController() {}

	void update() {}

private:

	/**
	*Controls the vehicle when following a list of points
	*/
	void executePointPath(const vehicle_auto_control::PointPathGoalConstPtr& goal, 
						  actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction>* as);

	void getTargetVelocityCommand(const vehicle_auto_control::Velocity vel);


	void sendVelocityCommand(double cmdForwardVelocity, double cmdLateralVelocity, double cmdRotVelocity, double cmdVertVelocity);

	/**
	* Checks to see if the vehicle is at a specific point with the error values
	* @param location Location of the vehicle
	* @param point Point to check
	*/
	bool isAtPoint(tf::Transform& location, tf::Vector3& point, bool useZ);

 	/**
	*Transforms the current point into the vehicle frame
	*@param pointOut Output point
	*@param transform Latest vehicle transform
	*/
	void transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform, tf::Vector3& point);

private:
	//Subscribers, publishers, and listeners
	ros::Subscriber velocitySub;
	ros::Publisher velocityPub;

	actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction> pointPathServer;
	tf::TransformListener listener;

	std::string currentTask;

	std::vector<tf::Vector3> pointPath;

	//YoYo Settings
	double yoyoUpperDepth;
	double yoyoLowerDepth;

	//Error bars for claiming the vehicle is at a point
	double lateralError;
	double verticalError;
	double rotationalError;

	//Target velocities for horizonal, vertical, and rotational movement
	double targetHorzVelocity;
	double targetRotVelocity;
	double targetVertVelocity;
};


#endif