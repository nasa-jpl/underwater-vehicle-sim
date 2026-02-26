#ifndef PROPULSION_LOGIC_INTERFACE_H
#define PROPULSION_LOGIC_INTERFACE_H

#include "ros/ros.h"

#include <memory>
#include <vector>

#include "geometry_msgs/Twist.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "ros_underwater_sim_utilities/VehiclePose.h"

class PropulsionLogicInterface
{

public:
	PropulsionLogicInterface(VehicleInfo& vehicleInfo);
	virtual ~PropulsionLogicInterface() {}

	virtual void processNewData(const underwater_vehicle_msgs::VehicleData data)=0;

	/**
	* Sends messages to make vehicle go to xy location
	*/
	virtual void goToXY(VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle follow a specified heading
	*/
	virtual void followHeading(VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle go to z location
	*/
	virtual void goToZ(VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle avoid the seafloor
	*/
	virtual void avoidSeafloor(VehiclePose& pose)=0;

	/**
	* Sends messages to stop xy movement
	*/
	virtual void stopXY()=0;

	/**
	* Sends messages to stop z movement
	*/
	virtual void stopZ()=0;

	/**
	* Sets the target xy location for goToXY
	*/
	void setTargetXY(double x, double y);
	
	/**
	* Sets the target z location for goToZ
	*/
	void setTargetZ(double z);

	/**
	* Sets the velocity in the xy direction
	*/
	void setVelocityXY(double xLinearVelocity, double yLinearVelocity, double zAngularVelocity);

	/**
	* Sets the velocity in the z direction
	*/
	void setVelocityZ(double zLinearVelocity);

	/**
	* Sets the target heading for followHeading
	*/
	virtual void setFollowHeading(double heading);

	/**
	* Gets the target x location for goToXY
	*/
	double getTargetX();

	/**
	* Gets the target y location for goToXY
	*/
	double getTargetY();
	
	/**
	* Gets the target z location for goToZ
	*/
	double getTargetZ();

	/**
	* Gets the target heading to follow 
	*/
	double getFollowHeading();

	/**
	* Determines if the vehicle has reached the xy location
	*/
	virtual bool isAtXY(VehiclePose& pose)=0;

	/**
	* Determines if the vehicle has reached the z location
	*/
	virtual bool isAtZ(VehiclePose& pose)=0;

protected:
	ros::NodeHandle vehicleNode;
	VehicleInfo& vehicleInfo;

	tf2::Vector3 targetLinearVelocity;
	tf2::Vector3 targetAngularVelocity;

	double targetX;
	double targetY;
	double targetZ;
	double targetHeading;

};

#endif