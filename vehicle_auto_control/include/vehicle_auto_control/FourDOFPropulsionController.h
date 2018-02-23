#ifndef FOUR_DOF_PROPULSION_CONTROLLER_H
#define FOUR_DOF_PROPULSION_CONTROLLER_H

#include "tf/transform_listener.h"

#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/PointPathAction.h"
#include "vehicle_auto_control/DynamicLawnmowerAction.h"

#include "underwater_vehicle_sim/VehicleData.h"

class FourDOFPropulsionController : public PropulsionController
{

public:
	FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName, float loopHertz);
	~FourDOFPropulsionController() {}

	void update() {}

private:

	/**
	*Controls the vehicle when following a list of points
	*/
	void executePointPath(const vehicle_auto_control::PointPathGoalConstPtr& goal, 
						  actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction>* as);
	
	void executeDynamicLawnmower(const vehicle_auto_control::DynamicLawnmowerGoalConstPtr& goal, 
						  			   actionlib::SimpleActionServer<vehicle_auto_control::DynamicLawnmowerAction>* as);

	/**
	*Processes the data for the dynamic lawnmower action
	*/
	bool processData(std::vector<float>& values, std::vector<double>& sectionAverages, double continueThreshold);

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

	tf::Vector3 getPoint(const tf::Vector3& startLocation, 
						 const double sectionSize, 
						 const double alongTrackDirection, 
						 const double acrossTrackDirection, 
						 const int currentTrack, 
						 const int currentSection);
private:
	//Subscribers, publishers, and listeners
	ros::Subscriber velocitySub;
	ros::Publisher velocityPub;

	actionlib::SimpleActionServer<vehicle_auto_control::DynamicLawnmowerAction> dynamicLawnmowerServer;
	actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction> pointPathServer;
	ros::ServiceClient plumeClient;
	tf::TransformListener listener;

	std::string currentTask;

	std::vector<tf::Vector3> pointPath;

	ros::Subscriber dataSub;
	double latestSonarDepth;
	double latestVehicleDepth;

	//YoYo Settings
	double yoyoUpperDepth;
	double yoyoLowerDepth;

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