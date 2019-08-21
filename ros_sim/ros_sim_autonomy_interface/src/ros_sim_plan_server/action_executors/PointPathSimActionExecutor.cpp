#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/GoToXYRosAction.h"

using namespace underwater_autonomy;

PointPathSimActionExecutor::PointPathSimActionExecutor(VehicleInfo& vehicleInfo) :
	PointPathSimActionExecutor(ros::NodeHandle(), vehicleInfo)
{}

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	goToXYClient(nh, "go_to_xy", false),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	stateAfterCancel(Action::State::INTERRUPTED),
	listener(buffer)
{
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &PointPathSimActionExecutor::navigationFilterCallback, this);
}

bool PointPathSimActionExecutor::execute(std::shared_ptr<PointPathAction> action)
{
	ROS_DEBUG("Execute point path action");

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

	//Creates an action goal and sends it to the action server for point path movement
	if(!action->isDone())
	{
		sendNextGoToXYGoal(action);
	}
	else
	{
		action->setState(Action::State::COMPLETED);
		ROS_DEBUG("Point path action completed");
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

	if(currentDuration.toSec() >= action->getTimeout())
	{
		//We need to cancel the action lib but still what to set the underwater autonomy action as failed
		stateAfterCancel = Action::State::FAILED;
		goToXYClient.cancelGoal();
	}

	if(!replanNextUpdate)
	{
		replanNextUpdate = action->doReplan(false,
											(ros::Time::now() - lastReplan).toSec(),
											distanceSinceReplan);
	}
}

void PointPathSimActionExecutor::cancel(std::shared_ptr<PointPathAction> action)
{
	action->setInterruptPoint(currentPose.getPosition());

	if(goToXYClient.getState() == actionlib::SimpleClientGoalState::PENDING ||
	   goToXYClient.getState() == actionlib::SimpleClientGoalState::ACTIVE)
	{
		goToXYClient.cancelAllGoals();
	}
	else
	{
		action->setState(Action::State::INTERRUPTED);
		ROS_DEBUG("point path action interrupted");
	}

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

void PointPathSimActionExecutor::actionDone(std::shared_ptr<PointPathAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::GoToXYRosResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		if(stateAfterCancel == Action::State::INTERRUPTED)
		{
			action->setState(Action::State::INTERRUPTED);
			ROS_DEBUG("Point Path action interrupted");
		}
		else if(stateAfterCancel == Action::State::FAILED)
		{
			action->setState(Action::State::FAILED);
			ROS_DEBUG("Point Path action failed");
		}
		else if(stateAfterCancel == Action::State::COMPLETED)
		{
			action->setState(Action::State::COMPLETED);
			ROS_DEBUG("Point Path action completed");
		}
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
		ROS_DEBUG("Point path action failed");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		bool replan = !action->getDoInterruptPoint();
		action->reachedTargetPoint();
		if(action->isDone())
		{
			action->setState(Action::State::COMPLETED);
			ROS_DEBUG("Point path action completed");
		}
		else
		{
			sendNextGoToXYGoal(action);
		}

		if(!replanNextUpdate)
		{
			replanNextUpdate = action->doReplan(replan,
												(ros::Time::now() - lastReplan).toSec(),
												distanceSinceReplan);
		}
	}
}

void PointPathSimActionExecutor::actionActive(std::shared_ptr<PointPathAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void PointPathSimActionExecutor::actionFeedback(std::shared_ptr<PointPathAction> action,
					const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback)
{}

void PointPathSimActionExecutor::sendNextGoToXYGoal(std::shared_ptr<underwater_autonomy::PointPathAction> action)
{
	//Creates an action goal and sends it to the action server for point path movement
	vehicle_auto_control::GoToXYRosGoal goToXYGoal;

	Eigen::Vector3d point = action->getCurrentTargetPoint();
	goToXYGoal.x = point[0];
	goToXYGoal.y = point[1];

	goToXYClient.waitForServer();
	ROS_DEBUG("Send goal to goToXY server");
	goToXYClient.sendGoal(goToXYGoal,
							 boost::bind(&PointPathSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&PointPathSimActionExecutor::actionActive, this, action),
							 boost::bind(&PointPathSimActionExecutor::actionFeedback, this, action, _1));
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