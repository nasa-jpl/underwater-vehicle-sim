#ifndef PROPULSION_CONTROLLER_H
#define PROPULSION_CONTROLLER_H

#include <memory>
#include <vector>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "vehicle_auto_control/GoToXYRosAction.h"
#include "vehicle_auto_control/GoToZRosAction.h"

class PropulsionController
{

public:
	PropulsionController(ros::NodeHandle& nh, VehicleInfo info);
	~PropulsionController() {}

	void update(void);

private:
	tf2::Stamped<tf2::Transform> getCurrentTransform();
	void getTargetVelocityCommand(const geometry_msgs::Twist vel);
	void getVehicleData(const underwater_vehicle_msgs::VehicleData data);

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

private:
	ros::NodeHandle controlNode;
	ros::NodeHandle vehicleNode;
	VehicleInfo info;

	std::unique_ptr<PropulsionLogicInterface> logicController;

	//Subscribers, publishers, and listeners
	ros::Subscriber velocitySub;
	ros::Publisher velocityPub;

	//Point Path Goal Parameters
	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer;
	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer;

	tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener;

	ros::Subscriber dataSub;

	
};

#endif