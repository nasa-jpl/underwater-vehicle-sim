#include "ros_sim_plan_server/action_executors/HoldDepthSimActionExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "underwater_autonomy/planner/actions/HoldDepthAction.h"

#include "actionlib/client/simple_action_client.h"

using namespace underwater_autonomy;

HoldDepthSimActionExecutor::HoldDepthSimActionExecutor(VehicleInfo& vehicleInfo) :
	HoldDepthSimActionExecutor(ros::NodeHandle(), vehicleInfo)
{}

HoldDepthSimActionExecutor::HoldDepthSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	goToZClient(nh, "go_to_z", true),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	currentDuration(0),
	stateAfterCancel(Action::State::INTERRUPTED),
	listener(buffer)
{
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &HoldDepthSimActionExecutor::navigationFilterCallback, this);
}

bool HoldDepthSimActionExecutor::execute(std::shared_ptr<HoldDepthAction> action)
{
	ROS_DEBUG("Execute hold depth action");

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

	//Creates an action goal and sends it to the action server for point path movement
	goToZGoal = vehicle_auto_control::GoToZRosGoal();

	goToZGoal.z = action->getDepth();
	goToZGoal.holdDepth = true;

	goToZClient.waitForServer();
	ROS_DEBUG("Send goal to GoToZ Server");
	goToZClient.sendGoal(goToZGoal,
						 boost::bind(&HoldDepthSimActionExecutor::rosActionDone, this, action, _1, _2),
						 boost::bind(&HoldDepthSimActionExecutor::rosActionActive, this, action),
						 boost::bind(&HoldDepthSimActionExecutor::rosActionFeedback, this, action, _1));
	
	lastUpdate = ros::Time::now();
	distanceSinceReplan = 0;
	return true;
}

void HoldDepthSimActionExecutor::cancel(std::shared_ptr<HoldDepthAction> action)
{
	goToZClient.cancelGoal();
}

bool HoldDepthSimActionExecutor::triggerReplan(std::shared_ptr<HoldDepthAction> action)
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

void HoldDepthSimActionExecutor::rosActionDone(std::shared_ptr<HoldDepthAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::GoToZRosResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		if(stateAfterCancel == Action::State::INTERRUPTED)
		{
			action->setState(Action::State::INTERRUPTED);
			ROS_DEBUG("Hold depth action interrupted");
		}
		else if(stateAfterCancel == Action::State::FAILED)
		{
			action->setState(Action::State::FAILED);
			ROS_DEBUG("Hold depth action failed");
		}
		else if(stateAfterCancel == Action::State::COMPLETED)
		{
			action->setState(Action::State::COMPLETED);
			ROS_DEBUG("Hold depth action completed");
		}
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
		ROS_DEBUG("Hold depth action failed");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
		ROS_DEBUG("Hold depth action completed from GoToZ return");
	}
}

void HoldDepthSimActionExecutor::rosActionActive(std::shared_ptr<HoldDepthAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void HoldDepthSimActionExecutor::rosActionFeedback(std::shared_ptr<HoldDepthAction> action,
					const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {}

void HoldDepthSimActionExecutor::monitor(std::shared_ptr<underwater_autonomy::HoldDepthAction> action)
{
	ros::Time currentTime = ros::Time::now();
	currentDuration += currentTime - lastUpdate;
	lastUpdate = currentTime;

	if(currentDuration.toSec() >= action->getHoldDepthTime())
	{
		//We need to cancel the action lib but still what to set the underwater autonomy action as complete
		stateAfterCancel = Action::State::COMPLETED;
		goToZClient.cancelGoal();
	}
	else if(currentDuration.toSec() >= action->getTimeout())
	{
		//We need to cancel the action lib but still what to set the underwater autonomy action as failed
		stateAfterCancel = Action::State::FAILED;
		goToZClient.cancelGoal();
	}

	replanNextUpdate = action->doReplan((ros::Time::now() - lastReplan).toSec(),
										distanceSinceReplan);
}

void HoldDepthSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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