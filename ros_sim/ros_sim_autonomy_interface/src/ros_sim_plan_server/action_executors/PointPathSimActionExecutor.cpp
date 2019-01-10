#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_planner/Action.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "vent_planner/actions/PointPathAction.h"

#include "actionlib/client/simple_action_client.h"
#include "ros_sim_plan_server/PointPathRosAction.h"

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh),
	pointPathClient("planner/"  + vehicleName + "/point_path", true),
	replanGoingUp(true),
	currentPointOffset(0),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0),
	listener(buffer)
{
	infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_msgs::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

PointPathSimActionExecutor::PointPathSimActionExecutor(const PointPathSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh),
	currentPointOffset(other.currentPointOffset),
	pointPathClient("planner/"  + vehicleName + "/point_path", true),
	replanNextUpdate(false),
	lastReplan(other.lastReplan),
	distanceSinceReplan(other.distanceSinceReplan),
	listener(buffer)
{
	infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_msgs::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

std::unique_ptr<ActionExecutor<PointPathAction>> PointPathSimActionExecutor::clone()
{
	std::unique_ptr<ActionExecutor<PointPathAction>> a(new PointPathSimActionExecutor(*this));
    return a;
}

bool PointPathSimActionExecutor::execute(std::shared_ptr<PointPathAction> action)
{
	ROS_INFO("Execute point path action");
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->getYoyo() && (action->getTargetSlope() >= M_PI / 2 || action->getTargetSlope() <= 0))
	{
		return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		std::string velSub = "vehicle_controller/" + vehicleName + "/command_target_velocity";
		if(!hasPublisher(velSub))
		{
			publishers.insert(std::make_pair(velSub, nh.advertise<geometry_msgs::Twist>(velSub, 1000, true)));
		}

		//Send target velocities command
		geometry_msgs::Twist velMsg;

		velMsg.linear.x = action->getTargetHorizontalVelocity();
		velMsg.linear.y = 0;
		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.linear.z = action->getTargetHorizontalVelocity() * (sin(action->getTargetSlope()) / cos(action->getTargetSlope()));

		velMsg.angular.x = 0;
		velMsg.angular.y = 0;
		velMsg.angular.z = action->getTargetRotationalVelocity();
	
		auto publisher = publishers.find(velSub); 
		publisher->second.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	try
	{
		geometry_msgs::TransformStamped transformMsg;
		tf2::Stamped<tf2::Transform> transform;
		if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(10.0)))
		{
			transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));
			tf2::fromMsg(transformMsg, transform);
			lastLocation = transform.getOrigin();
		}
		else
		{
			ROS_ERROR("No valid transform available");
		}		
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	//Creates an action goal and sends it to the action server for point path movement
	pointPathGoal = ros_sim_plan_server::PointPathRosGoal();

	if(action->getDoInterruptPoint())
	{
		Eigen::Vector3d& interruptPoint = action->getInterruptPoint();
		geometry_msgs::Point p;
		p.x = interruptPoint[0];
		p.y = interruptPoint[1];
		p.z = interruptPoint[2];
		pointPathGoal.points.push_back(p);
	}

	currentPointOffset = action->getCurrentPoint();
	for(unsigned int i = action->getCurrentPoint(); i < action->getPoints().size(); i++)
	{
		
		Eigen::Vector3d point = action->getPoints()[i];
		geometry_msgs::Point p;
		p.x = point[0];
		p.y = point[1];
		p.z = point[2];
		pointPathGoal.points.push_back(p);
	}

	//Set so the triggerReplan function knows when the yoyo direction changes
	replanGoingUp = true;

	pointPathGoal.upperDepth = action->getUpperDepth();
	pointPathGoal.lowerDepth = action->getLowerDepth();
	pointPathGoal.yoyo = action->getYoyo();

	pointPathClient.waitForServer();
	ROS_INFO("Send goal to point path server");
	pointPathClient.sendGoal(pointPathGoal,
							 boost::bind(&PointPathSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&PointPathSimActionExecutor::actionActive, this, action),
							 boost::bind(&PointPathSimActionExecutor::actionFeedback, this, action, _1));
	return true;
}

void PointPathSimActionExecutor::cancel(std::shared_ptr<PointPathAction> action)
{
	try
	{
		geometry_msgs::TransformStamped transformMsg;
		tf2::Stamped<tf2::Transform> transform;
		if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(10.0)))
		{
			transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));
			tf2::fromMsg(transformMsg, transform);
			action->setInterruptPoint(Eigen::Vector3d(transform.getOrigin().getX(), 
													  transform.getOrigin().getY(), 
													  transform.getOrigin().getZ()));
		}
		else
		{
			ROS_ERROR("No valid transform available");
		}		
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

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
                	const ros_sim_plan_server::PointPathRosResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action->setState(Action::State::INTERRUPTED);
		ROS_INFO("Point path action interrupted");
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
		ROS_INFO("Point path action failed");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
		ROS_INFO("Point path action completed");
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
					const ros_sim_plan_server::PointPathRosFeedbackConstPtr& feedback)
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
	
	//Check for replan
	if(action->getReplanType() == PointPathAction::ReplanType::ON_POINT_REACHED &&
	   action->getCurrentPoint() >= 1 && //at least at the first point
	   adjustedCurrentPoint != action->getCurrentPoint()) //reached a new point
	{
		replanNextUpdate = true;
	}
	else if(action->getReplanType() == PointPathAction::ReplanType::ON_YOYO_TURN && 
			action->getYoyo() && //insure we are yoyoing
			feedback->goingUp != action->getGoingUp() && //at top or bottom of yoyo
	   		action->getCurrentPoint() >= 1 && //at least at the first point
	   		(!action->getDoInterruptPoint() || (action->getDoInterruptPoint() && feedback->currentPoint >= 1))) //past the interrupt point
	{
		replanNextUpdate = true;
	}
	else if(action->getReplanType() == PointPathAction::ReplanType::PERIODIC_TIME &&
		    action->getCurrentPoint() >= 1 && 
		    (ros::Time::now() - lastReplan).toSec() > action->getPeriodicReplanValue())
	{
		replanNextUpdate = true;
	}
	else if(action->getReplanType() == PointPathAction::ReplanType::PERIODIC_DISTANCE &&
		    action->getCurrentPoint() >= 1)
	{
		try
		{
			geometry_msgs::TransformStamped transformMsg;
			tf2::Stamped<tf2::Transform> transform;
			if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(10.0)))
			{
				transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));
				tf2::fromMsg(transformMsg, transform);
				distanceSinceReplan += transform.getOrigin().distance(lastLocation);
				lastLocation = transform.getOrigin();
			}
			else
			{
				ROS_ERROR("No valid transform available");
			}		
		}
		catch(tf2::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}

		if(distanceSinceReplan >= action->getPeriodicReplanValue())
		{
			replanNextUpdate = true;
		}
	}

	if(adjustedCurrentPoint != action->getCurrentPoint())
	{
		//Wait until we have reached point 1 before any replanning
		ROS_DEBUG("Point Reached - Adjusted point: %i, Action Point: %i", adjustedCurrentPoint, action->getCurrentPoint());
		action->setCurrentPoint(adjustedCurrentPoint);
		action->addPointReachedTime(ros::Time::now().toSec());
	}

	action->setGoingUp(feedback->goingUp);
}

bool PointPathSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}