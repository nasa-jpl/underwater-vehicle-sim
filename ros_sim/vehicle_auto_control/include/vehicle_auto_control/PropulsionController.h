#ifndef PROPULSION_CONTROLLER_H
#define PROPULSION_CONTROLLER_H

#include <memory>
#include <vector>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "vehicle_auto_control/GoToXYRosAction.h"
#include "vehicle_auto_control/GoToZRosAction.h"
#include "vehicle_auto_control/FollowHeadingRosAction.h"

#include "underwater_autonomy/util/VehiclePose.h"

class PropulsionController
{

public:
	PropulsionController(VehicleInfo& info);
	PropulsionController(ros::NodeHandle nh, VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController);

	~PropulsionController() {}

	void update(void);

private:
	void navigationFilterCallback(const nav_msgs::Odometry odo);
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

	/**
	* Accepts new goals for the FollowHeading SimpleActionServer
	*/ 
	void goalFollowHeadingCB(void);
	void preemptFollowHeadingCB(void);
	void followHeadingUpdate(void);

private:

	ros::NodeHandle nh;

	VehicleInfo info;

	tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener;

	std::unique_ptr<PropulsionLogicInterface> logicController;

	//Point Path Goal Parameters
	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer;
	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer;
	actionlib::SimpleActionServer<vehicle_auto_control::FollowHeadingRosAction> followHeadingServer;

	ros::Time goToXYStart;

	ros::Time followHeadingStart;
	
	ros::Time goToZStart;
	bool holdAtZ;

	ros::Subscriber dataSub;

	ros::Subscriber velSub;
	ros::Subscriber poseSub;

	underwater_autonomy::VehiclePose currentPose;
};

#endif