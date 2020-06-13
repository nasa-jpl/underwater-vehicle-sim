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

#include "propulsion_controller/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"
#include "underwater_vehicle_msgs/GoToZ.h"
#include "underwater_vehicle_msgs/GoToXY.h"
#include "underwater_vehicle_msgs/FollowHeading.h"
#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/util/VehiclePose.h"


class PropulsionController
{

public:
	PropulsionController(ros::NodeHandle nh, VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController);
	PropulsionController(VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController);

	~PropulsionController() {}

	void update(void);

private:
	void navigationFilterCallback(const nav_msgs::Odometry odo);
	void getVehicleData(const underwater_vehicle_msgs::VehicleData data);

	//Go To XY
	bool goToXYCallback(underwater_vehicle_msgs::GoToXY::Request  &req,
                        underwater_vehicle_msgs::GoToXY::Response &res);
	void goToXYUpdate(void);

	//Go To Z
	bool goToZCallback(underwater_vehicle_msgs::GoToZ::Request  &req,
                       underwater_vehicle_msgs::GoToZ::Response &res);
	void goToZUpdate(void);

	//Follow Heading
	bool followHeadingCallback(underwater_vehicle_msgs::FollowHeading::Request  &req,
                               underwater_vehicle_msgs::FollowHeading::Response &res);
	void followHeadingUpdate(void);

	void publishState();
private:

	ros::NodeHandle nh;

	VehicleInfo info;

	std::unique_ptr<PropulsionLogicInterface> logicController;

	ros::Publisher statePub;

	//GoToZ Topics and Parameters
	ros::ServiceServer goToZService;
	bool goToZEnable;
	long zSeqNum;
	bool goToZComplete;
	bool goToZHoldDepth;

	//GoToXY Topics and Parameters
	ros::ServiceServer goToXYService;
	bool goToXYEnable;
	long xySeqNum;
	bool goToXYComplete;

	//FollowHeading Topics and Parameters
	ros::ServiceServer followHeadingService;
	bool followHeadingEnable;
	
	ros::Subscriber dataSub;

	ros::Subscriber poseSub;

	underwater_autonomy::VehiclePose currentPose;
};

#endif