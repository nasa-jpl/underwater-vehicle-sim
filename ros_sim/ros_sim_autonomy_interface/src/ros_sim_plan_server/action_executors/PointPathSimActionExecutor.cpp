#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/Action.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

#include "actionlib/client/simple_action_client.h"
#include "ros_sim_autonomy_interface/PointPathRosAction.h"

using namespace underwater_autonomy;

PointPathSimActionExecutor::PointPathSimActionExecutor(VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
	pointPathClient("point_path", true),
	currentPointOffset(0),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	listener(buffer)
{
	ros::NodeHandle nh;
	velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
	poseSub = nh.subscribe("primary_navigation", 1, &PointPathSimActionExecutor::navigationFilterCallback, this);

}

bool PointPathSimActionExecutor::execute(std::shared_ptr<PointPathAction> action)
{
	ROS_DEBUG("Execute point path action");
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->getYoyo() && (action->getTargetSlope() >= M_PI / 2 || action->getTargetSlope() <= 0))
	{
		return false;
	}

	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		//Send target velocities command
		geometry_msgs::Twist velMsg;

		velMsg.linear.x = action->getTargetHorizontalVelocity();
		velMsg.linear.y = 0;
		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.linear.z = action->getTargetHorizontalVelocity() * (sin(action->getTargetSlope()) / cos(action->getTargetSlope()));

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
	pointPathGoal = ros_sim_autonomy_interface::PointPathRosGoal();

	std::vector<Eigen::Vector3d> pointList = action->getCommandPoints();
	currentPointOffset = action->getCurrentPoint();
	for(unsigned int i = 0; i < pointList.size(); i++)
	{
		geometry_msgs::Point p;
		p.x = pointList[i][0];
		p.y = pointList[i][1];
		p.z = pointList[i][2];
		pointPathGoal.points.push_back(p);
	}

	pointPathGoal.upperDepth = action->getUpperDepth();
	pointPathGoal.lowerDepth = action->getLowerDepth();
	pointPathGoal.yoyo = action->getYoyo();

	pointPathClient.waitForServer();
	ROS_DEBUG("Send goal to point path server");
	pointPathClient.sendGoal(pointPathGoal,
							 boost::bind(&PointPathSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&PointPathSimActionExecutor::actionActive, this, action),
							 boost::bind(&PointPathSimActionExecutor::actionFeedback, this, action, _1));
	
	distanceSinceReplan = 0;
	return true;
}

void PointPathSimActionExecutor::cancel(std::shared_ptr<PointPathAction> action)
{
	action->setInterruptPoint(currentPose.getPosition());

	pointPathClient.cancelAllGoals();
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
                	const ros_sim_autonomy_interface::PointPathRosResultConstPtr& result)
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

	//Get the current point from the feedback
	unsigned int adjustedCurrentPoint = result->totalPoints;

	//If the interrupted point is active and we are past the 1st point then decrement the currentPoint
	if(adjustedCurrentPoint > 0 && action->getDoInterruptPoint())
	{
		adjustedCurrentPoint--;
	}
	//Add the currentPointOffset as we did not necessarily start at point 0
	adjustedCurrentPoint += currentPointOffset;
}

void PointPathSimActionExecutor::actionActive(std::shared_ptr<PointPathAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void PointPathSimActionExecutor::actionFeedback(std::shared_ptr<PointPathAction> action,
					const ros_sim_autonomy_interface::PointPathRosFeedbackConstPtr& feedback)
{
	//Get the current point from the feedback
	unsigned int adjustedCurrentPoint = feedback->currentPoint;

	//If the interrupted point is active and we are past the 1st point then decrement the currentPoint
	if(adjustedCurrentPoint > 0 && action->getDoInterruptPoint())
	{
		adjustedCurrentPoint--;
	}
	//Add the currentPointOffset as we did not necessarily start at point 0
	adjustedCurrentPoint += currentPointOffset;
	
	replanNextUpdate = action->doReplan(adjustedCurrentPoint,
										feedback->goingUp,
										(ros::Time::now() - lastReplan).toSec(),
										distanceSinceReplan);

	if(adjustedCurrentPoint != action->getCurrentPoint())
	{
		//Wait until we have reached point 1 before any replanning
		ROS_DEBUG("Point Reached - Adjusted point: %i, Action Point: %i", adjustedCurrentPoint, action->getCurrentPoint());
		action->setCurrentPoint(adjustedCurrentPoint);
		action->addPointReachedTime(ros::Time::now().toSec());
	}

	action->setGoingUp(feedback->goingUp);
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
	distanceSinceReplan += (position - currentPose.getPosition()).norm();


	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}