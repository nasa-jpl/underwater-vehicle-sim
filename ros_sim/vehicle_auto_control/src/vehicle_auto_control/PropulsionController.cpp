#include "vehicle_auto_control/PropulsionController.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

PropulsionController::PropulsionController(VehicleInfo& info) :
	PropulsionController(ros::NodeHandle(), info, PropulsionLogicInterface::makePropulsionLogic(info))
{}

PropulsionController::PropulsionController(ros::NodeHandle nh, VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController) :
	nh(nh),
	info(info),
	logicController(std::move(logicController)),
	goToZEnable(false),
	goToZHoldDepth(false),
	goToXYEnable(false),
	followHeadingEnable(false)
{
	std::vector<std::string> dataModuleNames = info.getModuleNamesOfType("DataBroadcaster");
	if(dataModuleNames.size() > 0)
    {
		//default to using first module of type DataBroadcaster if more than 1 exists
        dataSub = nh.subscribe(dataModuleNames[0] + "/data", 1, &PropulsionController::getVehicleData, this);
    }
	
    velSub = nh.subscribe("command_target_velocity", 10, &PropulsionController::getTargetVelocityCommand, this);
	poseSub = nh.subscribe("primary_navigation", 1, &PropulsionController::navigationFilterCallback, this);

	//Go To Z Topics
	goToZSub = nh.subscribe("go_to_z", 10, &PropulsionController::goToZCallback, this);
	goToZEnableSub = nh.subscribe("go_to_z_enable", 10, &PropulsionController::goToZEnableCallback, this);
	goToZComplete = nh.advertise<std_msgs::Bool>("go_to_z_complete", 1000);

	//Go To XY Topics
	goToXYSub = nh.subscribe("go_to_xy", 10, &PropulsionController::goToXYCallback, this);
	goToXYEnableSub = nh.subscribe("go_to_xy_enable", 10, &PropulsionController::goToXYEnableCallback, this);
	goToXYComplete = nh.advertise<std_msgs::Bool>("go_to_xy_complete", 1000);

	//Go To XY Topics
	followHeadingSub = nh.subscribe("follow_heading", 10, &PropulsionController::followHeadingCallback, this);
	followHeadingEnableSub = nh.subscribe("follow_heading_enable", 10, &PropulsionController::followHeadingEnableCallback, this);
	followHeadingComplete = nh.advertise<std_msgs::Bool>("follow_heading_complete", 1000);
}

void PropulsionController::getTargetVelocityCommand(const geometry_msgs::Twist vel)
{
	logicController->setTargetVelocity(vel);
}

void PropulsionController::getVehicleData(const underwater_vehicle_msgs::VehicleData data)
{
	logicController->processNewData(data);
}

void PropulsionController::update(void)
{
	//Only allow one type of XY commanding at a time.
	if(goToXYEnable)
    {
        goToXYUpdate();
    }
	else if(followHeadingEnable)
    {
        followHeadingUpdate();
    }

    if(goToZEnable)
    {
        goToZUpdate();
    }
	else
	{
		logicController->avoidSeafloor(currentPose);
	}
}

void PropulsionController::goToXYCallback(const underwater_vehicle_msgs::GoToXY parameters)
{
	logicController->setTargetXY(parameters.x, parameters.y);
	goToXYEnable = parameters.enable;
	if(goToXYEnable)
	{
		followHeadingEnable = false;
	}
}

void PropulsionController::goToXYEnableCallback(const std_msgs::Bool enable)
{
	if(enable.data)
	{
		goToXYEnable = true;
		followHeadingEnable = false;
	}
	else
	{
		logicController->stopXY();
		goToXYEnable = false;
	}
}

void PropulsionController::goToXYUpdate(void)
{	
	if(logicController->isAtXY(currentPose))
	{
		logicController->stopXY();
		goToXYEnable = false;

		std_msgs::Bool completeMsg;
		completeMsg.data = true;
		goToXYComplete.publish(completeMsg);
		ROS_DEBUG("GoToXY goal completed");
	}
	else
	{
		logicController->goToXY(currentPose);
	}
}

void PropulsionController::followHeadingCallback(const underwater_vehicle_msgs::FollowHeading parameters)
{
	followHeadingEnable = parameters.enable;
	if(followHeadingEnable)
	{
		goToXYEnable = false;
	}
	logicController->setFollowHeading(parameters.heading);
}

void PropulsionController::followHeadingEnableCallback(const std_msgs::Bool enable)
{
	if(enable.data)
	{
		followHeadingEnable = true;
		goToXYEnable = false;
	}
	else
	{
		logicController->stopXY();
		followHeadingEnable = false;
	}
}

void PropulsionController::followHeadingUpdate(void)
{	
	logicController->followHeading(currentPose);
}

void PropulsionController::goToZCallback(const underwater_vehicle_msgs::GoToZ parameters)
{
	goToZHoldDepth = parameters.holdDepth;
	goToZEnable = parameters.enable;
	logicController->setTargetZ(parameters.depth);
}

void PropulsionController::goToZEnableCallback(const std_msgs::Bool enable)
{
	if(enable.data)
	{
		goToZEnable = true;
	}
	else
	{
		logicController->stopZ();
		goToZEnable = false;
	}
}

void PropulsionController::goToZUpdate(void)
{
	if(logicController->isAtZ(currentPose) && !goToZHoldDepth)
	{
		logicController->stopZ();
		goToZEnable = false;

		std_msgs::Bool completeMsg;
		completeMsg.data = true;
		goToZComplete.publish(completeMsg);
		ROS_DEBUG("GoToZ goal completed");
	}
	else
	{
		logicController->goToZ(currentPose);
	}
}

void PropulsionController::navigationFilterCallback(const nav_msgs::Odometry odo)
{	
	Eigen::Vector3d position(odo.pose.pose.position.x,
							 odo.pose.pose.position.y,
							 odo.pose.pose.position.z);

	Eigen::Quaterniond orientation(odo.pose.pose.orientation.w,
								   odo.pose.pose.orientation.x,
								   odo.pose.pose.orientation.y,
								   odo.pose.pose.orientation.z);

	Eigen::Matrix<double,6,6> poseCovariance;
	for(unsigned int i = 0; i < 6; i++)
	{
		for(unsigned int j = 0; j < 6; j++)
		{
			poseCovariance(i, j) = odo.pose.covariance[(i * 6) + j];
		}
	}


	Eigen::Vector3d linearVelocity(odo.twist.twist.linear.x,
								   odo.twist.twist.linear.y,
								   odo.twist.twist.linear.z);
	Eigen::Vector3d angularVelocity(odo.twist.twist.angular.x,
								    odo.twist.twist.angular.y,
									odo.twist.twist.angular.z);

	Eigen::Matrix<double,6,6> twistCovariance;
	for(unsigned int i = 0; i < 6; i++)
	{
		for(unsigned int j = 0; j < 6; j++)
		{
			twistCovariance(i, j) = odo.twist.covariance[(i * 6) + j];
		}
	}

	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}