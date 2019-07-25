#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/Action.h"

#include "ros_sim_plan_server/action_executors/FollowHeadingSimActionExecutor.h"
#include "underwater_autonomy/planner/actions/FollowHeadingAction.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/FollowHeadingRosAction.h"

using namespace underwater_autonomy;

FollowHeadingSimActionExecutor::FollowHeadingSimActionExecutor(VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	followHeadingClient("follow_heading", true),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	listener(buffer)
{
	ros::NodeHandle nh;
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &FollowHeadingSimActionExecutor::navigationFilterCallback, this);
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

	//Creates an action goal and sends it to the action server for point path movement
	followHeadingGoal = vehicle_auto_control::FollowHeadingRosGoal();

	followHeadingGoal.heading = action->getHeading();
	followHeadingGoal.timeout = action->getTimeout();

	followHeadingClient.waitForServer();
	ROS_DEBUG("Send goal to point path server");
	followHeadingClient.sendGoal(followHeadingGoal,
							     boost::bind(&FollowHeadingSimActionExecutor::actionDone, this, action, _1, _2),
							     boost::bind(&FollowHeadingSimActionExecutor::actionActive, this, action),
							     boost::bind(&FollowHeadingSimActionExecutor::actionFeedback, this, action, _1));
	
	distanceSinceReplan = 0;
	return true;
}

void FollowHeadingSimActionExecutor::cancel(std::shared_ptr<FollowHeadingAction> action)
{
	followHeadingClient.cancelAllGoals();
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

void FollowHeadingSimActionExecutor::actionDone(std::shared_ptr<FollowHeadingAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::FollowHeadingRosResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action->setState(Action::State::INTERRUPTED);
		ROS_DEBUG("Point path action interrupted");
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
		ROS_DEBUG("Point path action failed");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
		ROS_DEBUG("Point path action completed");
	}
}

void FollowHeadingSimActionExecutor::actionActive(std::shared_ptr<FollowHeadingAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void FollowHeadingSimActionExecutor::actionFeedback(std::shared_ptr<FollowHeadingAction> action,
					const vehicle_auto_control::FollowHeadingRosFeedbackConstPtr& feedback)
{
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
	distanceSinceReplan += (position - currentPose.getPosition()).norm();


	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}