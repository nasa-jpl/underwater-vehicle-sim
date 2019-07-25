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

#include "underwater_autonomy/util/VehiclePose.h"

class PropulsionLogicInterface
{

public:
	PropulsionLogicInterface(VehicleInfo& vehicleInfo);
	virtual ~PropulsionLogicInterface() {}

	virtual void setTargetVelocity(const geometry_msgs::Twist vel)=0;
	virtual void processNewData(const underwater_vehicle_msgs::VehicleData data)=0;

	/**
	* Sends messages to make vehicle go to xy location
	*/
	virtual const void goToXY(underwater_autonomy::VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle follow a specified heading
	*/
	virtual const void followHeading(underwater_autonomy::VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle go to z location
	*/
	virtual const void goToZ(underwater_autonomy::VehiclePose& pose)=0;

	/**
	* Sends messages to make vehicle avoid the seafloor
	*/
	virtual const void avoidSeafloor(underwater_autonomy::VehiclePose& pose)=0;

	/**
	* Sends messages to stop xy movement
	*/
	virtual const void stopXY()=0;

	/**
	* Sends messages to stop z movement
	*/
	virtual const void stopZ()=0;

	/**
	* Sets the target xy location for goToXY
	*/
	virtual void setTargetXY(double x, double y)=0;
	
	/**
	* Sets the target z location for goToZ
	*/
	virtual void setTargetZ(double z)=0;

	/**
	* Sets the target headingtion for followHeading
	*/
	virtual void setFollowHeading(double heading)=0;

	/**
	* Determines if the vehicle has reached the xy location
	*/
	virtual bool isAtXY(underwater_autonomy::VehiclePose& pose)=0;

	/**
	* Determines if the vehicle has reached the z location
	*/
	virtual bool isAtZ(underwater_autonomy::VehiclePose& pose)=0;
	
	static std::unique_ptr<PropulsionLogicInterface> makePropulsionLogic(VehicleInfo& vehicleInfo);	

protected:
	ros::NodeHandle vehicleNode;
	VehicleInfo& vehicleInfo;
};

#endif