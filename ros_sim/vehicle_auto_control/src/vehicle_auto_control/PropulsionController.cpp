#include "vehicle_auto_control/PropulsionController.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

PropulsionController::PropulsionController(VehicleInfo& info) :
	PropulsionController(ros::NodeHandle(), info, PropulsionLogicInterface::makePropulsionLogic(info))
{}

PropulsionController::PropulsionController(ros::NodeHandle nh, VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController) :
	nh(nh),
	info(info),
	listener(buffer),
	logicController(std::move(logicController)),
	goToXYServer(nh, "go_to_xy", false),
	goToZServer(nh, "go_to_z", false),
	followHeadingServer(nh, "follow_heading", false),
	followHeadingTimeout(-1),
	goToZTimeout(-1),
	holdAtZ(false)
{
	std::vector<std::string> dataModuleNames = info.getModuleNamesOfType("DataBroadcaster");
	if(dataModuleNames.size() > 0)
    {
		//default to using first module of type DataBroadcaster if more than 1 exists
        dataSub = nh.subscribe(dataModuleNames[0] + "/data", 1, &PropulsionController::getVehicleData, this);
    }

    velSub = nh.subscribe("command_target_velocity", 1, &PropulsionController::getTargetVelocityCommand, this);
	poseSub = nh.subscribe("primary_navigation", 1, &PropulsionController::navigationFilterCallback, this);

	goToXYServer.registerGoalCallback(boost::bind(&PropulsionController::goalGoToXYCB, this));
    goToXYServer.registerPreemptCallback(boost::bind(&PropulsionController::preemptGoToXYCB, this));
 	goToXYServer.start();

    goToZServer.registerGoalCallback(boost::bind(&PropulsionController::goalGoToZCB, this));
    goToZServer.registerPreemptCallback(boost::bind(&PropulsionController::preemptGoToZCB, this));
	goToZServer.start();

	followHeadingServer.registerGoalCallback(boost::bind(&PropulsionController::goalFollowHeadingCB, this));
    followHeadingServer.registerPreemptCallback(boost::bind(&PropulsionController::preemptFollowHeadingCB, this));
	followHeadingServer.start();
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
	if(goToXYServer.isActive())
    {
        goToXYUpdate();
    }
	else if(followHeadingServer.isActive())
    {
        followHeadingUpdate();
    }

    if(goToZServer.isActive())
    {
        goToZUpdate();
    }
	else
	{
		logicController->avoidSeafloor(currentPose);
	}
}

/**
* Accepts new goals for the GoToXY SimpleActionServer
*/ 
void PropulsionController::goalGoToXYCB(void)
{
	logicController->stopXY();
	if(followHeadingServer.isActive())
	{
		followHeadingServer.setAborted();
	}

    vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
    
	logicController->setTargetXY(goToXYGoal->x, goToXYGoal->y);
    ROS_DEBUG("GoToXY server accepted a new goal - x:%f y:%f", goToXYGoal->x, goToXYGoal->y);
}

void PropulsionController::preemptGoToXYCB(void)
{
	logicController->stopXY();
    goToXYServer.setPreempted();
}

void PropulsionController::goToXYUpdate(void)
{	
	vehicle_auto_control::GoToXYRosFeedback feedback;
	feedback.x = currentPose.getPosition()[0];
	feedback.y = currentPose.getPosition()[1];
	goToXYServer.publishFeedback(feedback);

	if(logicController->isAtXY(currentPose))
	{
		logicController->stopXY();

		vehicle_auto_control::GoToXYRosResult result;
		result.x = currentPose.getPosition()[0];
		result.y = currentPose.getPosition()[1];
		goToXYServer.setSucceeded(result);
		ROS_DEBUG("GoToXY Server goal completed: %f %f", result.x, result.y);
	}
	else
	{
		logicController->goToXY(currentPose);
	}
}

void PropulsionController::goalFollowHeadingCB(void)
{
	logicController->stopXY();
	if(goToXYServer.isActive())
	{
		goToXYServer.setAborted();
	}

	vehicle_auto_control::FollowHeadingRosGoalConstPtr followHeadingGoal = followHeadingServer.acceptNewGoal();

	followHeadingStart = ros::Time::now();
	logicController->setFollowHeading(followHeadingGoal->heading);
	followHeadingTimeout = followHeadingGoal->timeout;

    ROS_DEBUG("FollowHeading server accepted a new goal - heading: %f", followHeadingGoal->heading);
}

void PropulsionController::preemptFollowHeadingCB(void)
{
	logicController->stopXY();
	
	followHeadingServer.setPreempted();
}

void PropulsionController::followHeadingUpdate(void)
{
	if(followHeadingTimeout > 0 && (ros::Time::now() - followHeadingStart).toSec() >= followHeadingTimeout)
	{
		logicController->stopXY();

		vehicle_auto_control::FollowHeadingRosResult result;
		followHeadingServer.setSucceeded(result);
		ROS_DEBUG("FollowHeading Server goal completed by timeout");
	}
	else
	{
		logicController->followHeading(currentPose);
	}
}

/**
* Accepts new goals for the GoToZ SimpleActionServer
*/ 
void PropulsionController::goalGoToZCB(void)
{
	logicController->stopZ();
    vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();

	goToZStart = ros::Time::now();
	logicController->setTargetZ(goToZGoal->z);
	followHeadingTimeout = goToZGoal->timeout;
	holdAtZ = goToZGoal->holdDepth;

    ROS_DEBUG("GoToZ server accepted a new goal - z: %f", goToZGoal->z);
}

void PropulsionController::preemptGoToZCB(void)
{
	logicController->stopZ();

    goToZServer.setPreempted();
}

void PropulsionController::goToZUpdate(void)
{
	vehicle_auto_control::GoToZRosFeedback feedback;
    feedback.z = currentPose.getPosition()[2];
    goToZServer.publishFeedback(feedback);

	if((goToZTimeout > 0 && (ros::Time::now() - goToZStart).toSec() >= goToZTimeout) ||
	   (logicController->isAtZ(currentPose) && !holdAtZ))
	{
		logicController->stopZ();
		vehicle_auto_control::GoToZRosResult result;
        result.z = currentPose.getPosition()[2];
        goToZServer.setSucceeded(result); 
		ROS_DEBUG("GoToZ Server goal completeted: %f", result.z);
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