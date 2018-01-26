#ifndef FOUR_DOF_PROPULSION_CONTROLLER_H
#define FOUR_DOF_PROPULSION_CONTROLLER_H

#include "tf/transform_listener.h"

#include "vehicle_auto_control/PropulsionController.h"

#include "vehicle_auto_control/PointPath.h"
#include "vehicle_auto_control/YoYoPointPath.h"
#include "vehicle_auto_control/Velocity.h"

class FourDOFPropulsionController : public PropulsionController
{

public:
	FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName);
	virtual ~FourDOFPropulsionController() {}


	void update();

	
private:
	/**
	*Controls the vehicle when following a list of points
	*/
	void pointPathController();

	/**
	*Controls the vehicle when following a list of points while yoyoing
	*/
	void yoyoPointPathController();

	void getPointPathCommand(const vehicle_auto_control::PointPath vel);
	void getYoYoPointPathCommand(const vehicle_auto_control::YoYoPointPath vel);
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
	void transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform);

private:
	//Subscribers, publishers, and listeners
	ros::Subscriber commandPointPathSub;
	ros::Subscriber commandYoYoPointPathSub;
	ros::Subscriber commandTargetVelocitySub;
	ros::Publisher velocityPub;
	tf::TransformListener listener;

	std::string currentTask;

	std::vector<tf::Vector3> pointPath;
	unsigned int currentPoint;

	//YoYo Settings
	double yoyoUpperDepth;
	double yoyoLowerDepth;
	bool goingUp;

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