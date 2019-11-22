#ifndef PROPULSION_CONTROLLER_H
#define PROPULSION_CONTROLLER_H

#include <memory>
#include <vector>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"
#include "underwater_vehicle_msgs/GoToZ.h"
#include "underwater_vehicle_msgs/GoToXY.h"
#include "underwater_vehicle_msgs/FollowHeading.h"

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

	//Go To XY
	void goToXYCallback(const underwater_vehicle_msgs::GoToXY parameters);
	void goToXYEnableCallback(const std_msgs::Bool enable);
	void goToXYUpdate(void);

	//Go To Z
	void goToZCallback(const underwater_vehicle_msgs::GoToZ parameters);
	void goToZEnableCallback(const std_msgs::Bool enable);
	void goToZUpdate(void);

	//Follow Heading
	void followHeadingCallback(const underwater_vehicle_msgs::FollowHeading parameters);
	void followHeadingEnableCallback(const std_msgs::Bool enable);
	void followHeadingUpdate(void);

private:

	ros::NodeHandle nh;

	VehicleInfo info;

	std::unique_ptr<PropulsionLogicInterface> logicController;

	//GoToZ Topics and Parameters
	ros::Subscriber goToZSub;
	ros::Subscriber goToZEnableSub;
	ros::Publisher goToZComplete;
	bool goToZEnable;
	bool goToZHoldDepth;

	//GoToXY Topics and Parameters
	ros::Subscriber goToXYSub;
	ros::Subscriber goToXYEnableSub;
	ros::Publisher goToXYComplete;
	bool goToXYEnable;

	//FollowHeading Topics and Parameters
	ros::Subscriber followHeadingSub;
	ros::Subscriber followHeadingEnableSub;
	ros::Publisher followHeadingComplete;
	bool followHeadingEnable;
	
	ros::Subscriber dataSub;

	ros::Subscriber velSub;
	ros::Subscriber poseSub;

	underwater_autonomy::VehiclePose currentPose;
};

#endif