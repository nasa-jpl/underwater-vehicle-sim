#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"

#include "vent_planner/executors/PointPathSimActionExecutor.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathRosAction.h"

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh),
	pointPathClient("vehicle_controller/"  + vehicleName + "/point_path", true),
	replanGoingUp(true),
	currentPointOffset(0),
	replanNextUpdate(false),
	lastReplan(ros::Time::now()),
	distanceSinceReplan(0)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

PointPathSimActionExecutor::PointPathSimActionExecutor(const PointPathSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh),
	currentPointOffset(other.currentPointOffset),
	pointPathClient("vehicle_controller/"  + vehicleName + "/point_path", true),
	replanNextUpdate(false),
	lastReplan(other.lastReplan),
	distanceSinceReplan(other.distanceSinceReplan)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
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
			publishers.insert(std::make_pair(velSub, nh.advertise<vehicle_auto_control::Velocity>(velSub, 1000, true)));
		}

		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action->getTargetHorizontalVelocity();

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action->getTargetHorizontalVelocity() * (sin(action->getTargetSlope()) / cos(action->getTargetSlope()));

		velMsg.rotationalVelocity = action->getTargetRotationalVelocity();
	
		auto publisher = publishers.find(velSub); 
		publisher->second.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	try
	{
		tf::StampedTransform transform;
		listener.waitForTransform("/world", "/" + vehicleName, ros::Time(0), ros::Duration(5.0));
		listener.lookupTransform("/world", "/" + vehicleName, ros::Time(0), transform);
		lastLocation = transform.getOrigin();
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	//Creates an action goal and sends it to the action server for point path movement
	pointPathGoal = vehicle_auto_control::PointPathRosGoal();

	if(action->getDoInterruptPoint())
	{
		tf::Vector3& interruptPoint = action->getInterruptPoint();
		geometry_msgs::Point p;
		p.x = interruptPoint.getX();
		p.y = interruptPoint.getY();
		p.z = interruptPoint.getZ();
		pointPathGoal.points.push_back(p);
	}

	currentPointOffset = action->getCurrentPoint();
	for(unsigned int i = action->getCurrentPoint(); i < action->getPoints().size(); i++)
	{
		auto point = action->getPoints()[i];
		geometry_msgs::Point p;
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
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
		tf::StampedTransform transform;
		listener.waitForTransform("/world", "/" + vehicleName, ros::Time(0), ros::Duration(5.0));
		listener.lookupTransform("/world", "/" + vehicleName, ros::Time(0), transform);
		action->setInterruptPoint(transform.getOrigin());
	}
	catch (tf::TransformException ex)
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
                	const vehicle_auto_control::PointPathRosResultConstPtr& result)
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
					const vehicle_auto_control::PointPathRosFeedbackConstPtr& feedback)
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
			tf::StampedTransform transform;
			listener.waitForTransform("/world", "/" + vehicleName, ros::Time(0), ros::Duration(5.0));
			listener.lookupTransform("/world", "/" + vehicleName, ros::Time(0), transform);
			distanceSinceReplan += transform.getOrigin().distance(lastLocation);
			lastLocation = transform.getOrigin();
		}
		catch (tf::TransformException ex)
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
		action->addPointReachedTime(ros::Time::now());
	}

	action->setGoingUp(feedback->goingUp);
}

bool PointPathSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}