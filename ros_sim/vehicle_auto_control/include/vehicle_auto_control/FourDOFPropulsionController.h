#ifndef FOUR_DOF_PROPULSION_CONTROLLER_H
#define FOUR_DOF_PROPULSION_CONTROLLER_H

#include "tf/transform_listener.h"

#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/GoToXYRosAction.h"
#include "vehicle_auto_control/GoToZRosAction.h"

#include "underwater_vehicle_msgs/VehicleData.h"

class FourDOFPropulsionController : public PropulsionController
{

public:
	FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName);
	~FourDOFPropulsionController() {}

	void update(void);
private:

	/**
	* Accepts new goals for the GoToXY SimpleActionServer
	*/ 
	void goalGoToXYCB(void);
	void preemptGoToXYCB(void);
	void goToXYUpdate(void);

	/**
	* Accepts new goals for the GoToZ SimpleActionServer
	*/ 
	void goalGoToZCB(void);
	void preemptGoToZCB(void);
	void goToZUpdate(void);


	void getTargetVelocityCommand(const vehicle_auto_control::Velocity vel);

	void getVehicleData(const underwater_vehicle_msgs::VehicleData data);

	void sendVelocityCommand(double cmdForwardVelocity, double cmdLateralVelocity, double cmdRotVelocity, double cmdVertVelocity);


	bool isAtXY(tf::Transform& location);
	bool isAtZ(tf::Transform& location);

	double scaleHorizontalVelocity(tf::Transform& location, tf::Vector3& point);
	double scaleVerticalVelocity(tf::Transform& location, double targetHeight);
	double scaleRotationalVelocity(double angleError, double crossZ);

	/**
	*Sends the needed commands to go toward the specified xy location
	*/
	void goToXY(tf::StampedTransform& location);

	/**
	*Sends the needed commands to go toward the specified z location
	*/
	void goToZ(tf::StampedTransform& location);
 	
 	/**
	*Transforms the current point into the vehicle frame
	*@param pointOut Output point
	*@param transform Latest vehicle transform
	*/
	void transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform, tf::Vector3& point);

	/**
	*Cancels all actionlib goals related to vehicle movements in xy direction
	*/
	void cancelXYMovement(void);
	/**
	*Cancels all actionlib goals related to vehicle movements in z direction
	*/
	void cancelZMovement(void);

private:
	//Subscribers, publishers, and listeners
	ros::Subscriber velocitySub;
	ros::Publisher velocityPub;

	//Point Path Goal Parameters
	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer;
	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer;

	double targetX;
	double targetY;
	double targetZ;

	//Track the last sent velocity commands so we can update xy and z independently
	double lastForwardVelocity;
	double lastLateralVelocity;
	double lastRotVelocity;
	double lastVertVelocity;

	tf::TransformListener listener;

	ros::Subscriber dataSub;
	double latestSonarDepth;
	double latestVehicleDepth;

	//Error bars for claiming the vehicle is at a point
	double lateralError;
	double verticalError;

	//Target velocities for horizonal, vertical, and rotational movement
	double targetHorzVelocity;
	double targetRotVelocity;
	double targetVertVelocity;

	//Minimum distance off seafloor
	double minSeafloorDistance;
	bool hasVehicleData;
};


#endif