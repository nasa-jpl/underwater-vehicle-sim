#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/FollowHeading.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "ros_sim_plan_server/action_executors/FollowHeadingSimActionExecutor.h"

using namespace underwater_autonomy;

FollowHeadingSimActionExecutor::FollowHeadingSimActionExecutor(VehicleInfo& vehicleInfo) :
	FollowHeadingSimActionExecutor(ros::NodeHandle(), vehicleInfo)
{}

FollowHeadingSimActionExecutor::FollowHeadingSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	currentDuration(0),
	gotCompleteCallback(false)
{
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &FollowHeadingSimActionExecutor::navigationFilterCallback, this);

	followHeadingPub = nh.advertise<underwater_vehicle_msgs::FollowHeading>("follow_heading", 1000);
	followHeadingEnablePub = nh.advertise<std_msgs::Bool>("follow_heading_enable", 1000);
	followHeadingComplete = nh.subscribe("follow_heading_complete", 1, &FollowHeadingSimActionExecutor::followHeadingCompleteCallback, this);
}

bool FollowHeadingSimActionExecutor::execute(std::shared_ptr<FollowHeadingAction> action)
{
	ROS_DEBUG("Execute follow heading action");

	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		//Send target velocities command
		geometry_msgs::Twist velMsg;

		velMsg.linear.x = action->getTargetHorizontalVelocity();
		velMsg.linear.y = 0;

        //z is set to nan as we do not want to modify it
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
	while(followHeadingPub.getNumSubscribers() == 0 &&
		  ros::WallTime::now() - time < ros::WallDuration(5)) {ros::WallDuration(1).sleep();}
	if(followHeadingPub.getNumSubscribers() == 0)
	{
		return false;
	}

	//Send message to Follow Heading Controller
	//Reset complete callback
	gotCompleteCallback = false;
	underwater_vehicle_msgs::FollowHeading followHeadingMsg;
	followHeadingMsg.heading = action->getHeading();
	followHeadingMsg.enable = true;
	followHeadingPub.publish(followHeadingMsg);
	action->setState(Action::State::EXECUTING);

	lastUpdate = ros::Time::now();
	distanceSinceReplan = 0;
	return true;
}

void FollowHeadingSimActionExecutor::cancel(std::shared_ptr<FollowHeadingAction> action)
{
	std_msgs::Bool enableMsg;
	enableMsg.data = false;
	followHeadingEnablePub.publish(enableMsg);

	action->setState(Action::State::INTERRUPTED);
}

bool FollowHeadingSimActionExecutor::triggerReplan(std::shared_ptr<FollowHeadingAction> action)
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

void FollowHeadingSimActionExecutor::followHeadingCompleteCallback(const std_msgs::Bool complete)
{
	if(complete.data)
	{
		gotCompleteCallback = true;
	}
}

void FollowHeadingSimActionExecutor::monitor(std::shared_ptr<underwater_autonomy::FollowHeadingAction> action)
{
	ros::Time currentTime = ros::Time::now();
	currentDuration += currentTime - lastUpdate;
	lastUpdate = currentTime;

	if(gotCompleteCallback && action->getState() == Action::State::EXECUTING)
	{
		gotCompleteCallback = false;

		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		followHeadingEnablePub.publish(enableMsg);

		action->setState(Action::State::COMPLETED);
	}
	else if(action->getFollowHeadingTime() >= 0 && currentDuration.toSec() >= action->getFollowHeadingTime())
	{
		action->setState(Action::State::COMPLETED);

		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		followHeadingEnablePub.publish(enableMsg);
	}
	else if(action->getTimeout() >= 0 && currentDuration.toSec() >= action->getTimeout())
	{
		action->setState(Action::State::FAILED);

		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		followHeadingEnablePub.publish(enableMsg);
	}
	else if(!action->inXYOperationRegion(currentPose.getPosition()))
	{
		action->setState(Action::State::FAILED);

		std_msgs::Bool enableMsg;
		enableMsg.data = false;
		followHeadingEnablePub.publish(enableMsg);
	}

	replanNextUpdate = action->doReplan((ros::Time::now() - lastReplan).toSec(),
										distanceSinceReplan);
}

void FollowHeadingSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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