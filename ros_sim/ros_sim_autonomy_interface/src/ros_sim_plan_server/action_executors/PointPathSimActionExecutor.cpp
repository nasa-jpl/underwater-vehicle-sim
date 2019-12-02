#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToXY.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

using namespace underwater_autonomy;

PointPathSimActionExecutor::PointPathSimActionExecutor(VehicleInfo& vehicleInfo) :
	PointPathSimActionExecutor(ros::NodeHandle(), vehicleInfo)
{}

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0)
{
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &PointPathSimActionExecutor::navigationFilterCallback, this);

	goToXYPub = nh.advertise<underwater_vehicle_msgs::GoToXY>("go_to_xy", 1000);
	goToXYEnablePub = nh.advertise<std_msgs::Bool>("go_to_xy_enable", 1000);
	goToXYComplete = nh.subscribe("go_to_xy_complete", 1, &PointPathSimActionExecutor::goToXYCompleteCallback, this);
}

bool PointPathSimActionExecutor::execute(std::shared_ptr<PointPathAction> action)
{
	ROS_INFO("Execute point path action");

	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		//Send target velocities command
		geometry_msgs::Twist velMsg;

		velMsg.linear.x = action->getTargetHorizontalVelocity();
		velMsg.linear.y = 0;
		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.linear.z = std::numeric_limits<double>::quiet_NaN();

		velMsg.angular.x = 0;
		velMsg.angular.y = 0;
		velMsg.angular.z = action->getTargetRotationalVelocity();
	
		velPub.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	//Check that we have someone listening to us
	ros::WallTime time = ros::WallTime::now();
	while(goToXYPub.getNumSubscribers() == 0 &&
		  ros::WallTime::now() - time < ros::WallDuration(5)) {ros::WallDuration(1).sleep();}
	if(goToXYPub.getNumSubscribers() == 0)
	{
		return false;
	}


	//Creates an action goal and sends it to the action server for point path movement
	if(!action->isDone())
	{
		sendNextGoToXYGoal(action);
		action->setState(Action::State::EXECUTING);
	}
	else
	{
		action->setState(Action::State::COMPLETED);
		ROS_INFO("Point path action completed");
	}
	
	lastUpdate = ros::Time::now();
	distanceSinceReplan = 0;
	return true;
}

void PointPathSimActionExecutor::monitor(std::shared_ptr<underwater_autonomy::PointPathAction> action)
{
	ros::Time currentTime = ros::Time::now();
	currentDuration += currentTime - lastUpdate;
	lastUpdate = currentTime;

	if(gotCompleteCallback && action->getState() == Action::State::EXECUTING)
	{
		gotCompleteCallback = false;
		action->reachedTargetPoint();
		if(action->isDone())
		{
			action->setState(Action::State::COMPLETED);
			ROS_INFO("Point path action completed");
		}
		else
		{
			sendNextGoToXYGoal(action);
		}
	}
	else if(action->getTimeout() >= 0 && currentDuration.toSec() >= action->getTimeout())
	{
		action->setState(Action::State::FAILED);
		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		goToXYEnablePub.publish(enableMsg);
	}

	if(!replanNextUpdate)
	{
		bool replan = !action->getDoInterruptPoint();
		replanNextUpdate = action->doReplan(replan,
											(ros::Time::now() - lastReplan).toSec(),
											distanceSinceReplan);
	}
}

void PointPathSimActionExecutor::cancel(std::shared_ptr<PointPathAction> action)
{
	action->setState(Action::State::INTERRUPTED);
	action->setInterruptPoint(currentPose.getPosition());

	std_msgs::Bool enableMsg;
	enableMsg.data = false;
	goToXYEnablePub.publish(enableMsg);

	ROS_INFO("point path action interrupted");
}

bool PointPathSimActionExecutor::triggerReplan(std::shared_ptr<PointPathAction> action)
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

void PointPathSimActionExecutor::goToXYCompleteCallback(const std_msgs::Bool complete)
{
	if(complete.data)
	{
		gotCompleteCallback = true;
	}
}

void PointPathSimActionExecutor::sendNextGoToXYGoal(std::shared_ptr<underwater_autonomy::PointPathAction> action)
{
	//Reset the gotCompleteCallback as this might have tripped on previous actions
	gotCompleteCallback = false;

	Eigen::Vector3d point = action->getCurrentTargetPoint();
	underwater_vehicle_msgs::GoToXY goToXYMsg;
	goToXYMsg.x = point[0];
	goToXYMsg.y = point[1];
	goToXYMsg.enable = true;
	
	goToXYPub.publish(goToXYMsg);
}

void PointPathSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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
	zeroedPosition[2] = 0;
	zeroedCurrentPosition[2] = 0;
	distanceSinceReplan += (zeroedPosition - zeroedCurrentPosition).norm();

	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}