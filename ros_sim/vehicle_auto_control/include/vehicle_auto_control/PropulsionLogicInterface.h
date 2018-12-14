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

class PropulsionLogicInterface
{

public:
	PropulsionLogicInterface() {};
	virtual ~PropulsionLogicInterface() {}

	virtual void setTargetVelocity(const geometry_msgs::Twist vel)=0;
	virtual void processNewData(const underwater_vehicle_msgs::VehicleData data)=0;

	virtual const geometry_msgs::Twist goToXYTwist(tf2::Stamped<tf2::Transform>& NEDToVehicle)=0;
	virtual const geometry_msgs::Twist goToZTwist(tf2::Stamped<tf2::Transform>& NEDToVehicle)=0;

	virtual const geometry_msgs::Twist stopXYTwist()=0;
	virtual const geometry_msgs::Twist stopZTwist()=0;

	virtual void setTargetXY(double x, double y)=0;
	virtual void setTargetZ(double z)=0;

	virtual bool isAtXY(tf2::Stamped<tf2::Transform>& transform)=0;
	virtual bool isAtZ(tf2::Stamped<tf2::Transform>& transform)=0;
	
	static std::unique_ptr<PropulsionLogicInterface> makePropulsionLogic(VehicleInfo info);	

private:

private:

};

#endif