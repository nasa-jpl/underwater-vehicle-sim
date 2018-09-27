#ifndef FOUR_DOF_PROPULSION_CONTROLLER_H
#define FOUR_DOF_PROPULSION_CONTROLLER_H

#include "tf/transform_listener.h"

#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/PointPathRosAction.h"
#include "vehicle_auto_control/CirclePathRosAction.h"


#include "underwater_vehicle_sim/VehicleData.h"

class FourDOFPropulsionController : public PropulsionController
{

public:
	FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName);
	~FourDOFPropulsionController() {}

	void update(void);
private:

	/**
	* Accepts new goals for the PointPath SimpleActionServer
	*/ 
	void goalPointPathCB(void);
	void preemptPointPathCB(void);
	void pointPathUpdate(void);

	void getTargetVelocityCommand(const vehicle_auto_control::Velocity vel);

	void getVehicleData(const underwater_vehicle_sim::VehicleData data);

	void sendVelocityCommand(double cmdForwardVelocity, double cmdLateralVelocity, double cmdRotVelocity, double cmdVertVelocity);

	/**
	* Checks to see if the vehicle is at a specific point with the error values
	* @param location Location of the vehicle
	* @param point Point to check
	*/
	bool isAtPoint(tf::Transform& location, tf::Vector3& point, bool useZ);

	double scaleHorizontalVelocity(tf::Transform& location, tf::Vector3& point);
	double scaleVerticalVelocity(tf::Transform& location, double targetHeight);
	double scaleRotationalVelocity(double angleError, double crossZ);

	/**
	*Sends the needed commands to go toward the specified point
	*
	*/
	void goToPoint(tf::StampedTransform& location, tf::Vector3& point, double targetHeight);
 	
 	/**
	*Transforms the current point into the vehicle frame
	*@param pointOut Output point
	*@param transform Latest vehicle transform
	*/
	void transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform, tf::Vector3& point);

	/**
	*Cancels all actionlib goals related to vehicle movements
	*/
	void cancelAllMovement(void);

private:
	//Subscribers, publishers, and listeners
	ros::Subscriber velocitySub;
	ros::Publisher velocityPub;

	//Point Path Goal Parameters
	actionlib::SimpleActionServer<vehicle_auto_control::PointPathRosAction> pointPathServer;
	std::vector<tf::Vector3> pathPoints;
	unsigned int currentPoint;

	//Shared Parameters
	bool yoyo;
	int upperDepth;
	int lowerDepth;

	//State variables
    bool goingUp;

	ros::ServiceClient plumeClient;
	tf::TransformListener listener;

	std::vector<tf::Vector3> pointPath;

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