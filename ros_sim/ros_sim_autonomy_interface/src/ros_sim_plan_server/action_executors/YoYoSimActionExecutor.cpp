#include "ros_sim_plan_server/action_executors/YoYoSimActionExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToZ.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "underwater_autonomy/planner/actions/YoYoAction.h"

using namespace underwater_autonomy;

YoYoSimActionExecutor::YoYoSimActionExecutor(VehicleInfo& vehicleInfo) :
	YoYoSimActionExecutor(ros::NodeHandle(), vehicleInfo)
{}

YoYoSimActionExecutor::YoYoSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	currentDuration(0),
	gotCompleteCallback(false)
{
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &YoYoSimActionExecutor::navigationFilterCallback, this);

	goToZPub = nh.advertise<underwater_vehicle_msgs::GoToZ>("go_to_z", 1000);
	goToZEnablePub = nh.advertise<std_msgs::Bool>("go_to_z_enable", 1000);
	goToZComplete = nh.subscribe("go_to_z_complete", 1, &YoYoSimActionExecutor::goToZCompleteCallback, this);

}

bool YoYoSimActionExecutor::execute(std::shared_ptr<YoYoAction> action)
{
	ROS_INFO("Execute yoyo action");

	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		//Send target velocities command
		geometry_msgs::Twist velMsg;

	    //xy is set to nan as we do not want to modify it
		velMsg.linear.x = std::numeric_limits<double>::quiet_NaN();
		velMsg.linear.y = std::numeric_limits<double>::quiet_NaN();

		velMsg.linear.z = action->getTargetVerticalVelocity();

		velMsg.angular.x = std::numeric_limits<double>::quiet_NaN();
		velMsg.angular.y = std::numeric_limits<double>::quiet_NaN();
		velMsg.angular.z = std::numeric_limits<double>::quiet_NaN();
	
		velPub.publish(velMsg);
	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}


	//Check that we have someone listening to us
	ros::WallTime time = ros::WallTime::now();
	while(goToZPub.getNumSubscribers() == 0 &&
		  ros::WallTime::now() - time < ros::WallDuration(5)) {ros::WallDuration(1).sleep();}
	if(goToZPub.getNumSubscribers() == 0)
	{
		return false;
	}


	//Creates an action goal and sends it to the action server for point path movement
	sendNewGoToZGoal(action);
	action->setState(Action::State::EXECUTING);

	distanceSinceReplan = 0;
	lastUpdate = ros::Time::now();
	return true;
}

void YoYoSimActionExecutor::cancel(std::shared_ptr<YoYoAction> action)
{
	std_msgs::Bool enableMsg;
	enableMsg.data = false;
	goToZEnablePub.publish(enableMsg);

	action->setState(Action::State::INTERRUPTED);
}

bool YoYoSimActionExecutor::triggerReplan(std::shared_ptr<YoYoAction> action)
{
	if(replanNextUpdate)
	{
		replanNextUpdate = false;
		lastReplan = ros::Time::now();
		distanceSinceReplan = 0;
		return true;
	}

	return false;
}

void YoYoSimActionExecutor::goToZCompleteCallback(const std_msgs::Bool complete)
{
	if(complete.data)
	{		
		gotCompleteCallback = true;
	}
}

void YoYoSimActionExecutor::monitor(std::shared_ptr<underwater_autonomy::YoYoAction> action)
{
	ros::Time currentTime = ros::Time::now();
	currentDuration += currentTime - lastUpdate;
	lastUpdate = currentTime;

	if(gotCompleteCallback && action->getState() == Action::State::EXECUTING)
	{
		gotCompleteCallback = false;

		action->setGoingUp(!action->getGoingUp());
		if(!replanNextUpdate)
		{

			replanNextUpdate = action->doReplan(true, (ros::Time::now() - lastReplan).toSec(),
												distanceSinceReplan);
		}

		if(action->getState() == Action::State::EXECUTING)
		{
			sendNewGoToZGoal(action);
		}
	}
	else if(action->getYoYoTime() >= 0 && currentDuration.toSec() >= action->getYoYoTime())
	{
		action->setState(Action::State::COMPLETED);
		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		goToZEnablePub.publish(enableMsg);
	}
	else if(action->getTimeout() >= 0 && currentDuration.toSec() >= action->getTimeout())
	{
		action->setState(Action::State::FAILED);
		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		goToZEnablePub.publish(enableMsg);
	}

	if(!replanNextUpdate)
	{
		replanNextUpdate = action->doReplan(false, (ros::Time::now() - lastReplan).toSec(),
											distanceSinceReplan);
	}
}

void YoYoSimActionExecutor::sendNewGoToZGoal(std::shared_ptr<underwater_autonomy::YoYoAction> action)
{
	//Reset got complete callback
	gotCompleteCallback = false;
	underwater_vehicle_msgs::GoToZ goToZMsg;
	if(action->getGoingUp())
	{
		goToZMsg.depth = action->getUpperDepth();
	}
	else
	{
		goToZMsg.depth = action->getLowerDepth();
	}

	goToZMsg.enable = true;
	goToZMsg.holdDepth = false;

	goToZPub.publish(goToZMsg);
}

void YoYoSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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

	//Update the distance since replanning
	Eigen::Vector3d zeroedPosition = position;
	Eigen::Vector3d zeroedCurrentPosition = currentPose.getPosition();
	zeroedPosition[0] = 0;
	zeroedPosition[1] = 0;
	zeroedCurrentPosition[0] = 0;
	zeroedCurrentPosition[1] = 0;
	distanceSinceReplan += (zeroedPosition - zeroedCurrentPosition).norm();

	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}