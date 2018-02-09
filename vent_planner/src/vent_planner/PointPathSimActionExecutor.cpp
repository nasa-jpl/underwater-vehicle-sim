#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"

#include "vent_planner/PointPathSimActionExecutor.h"
#include "vent_planner/actions/PointPathAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh),
	pointPathClient("/vehicle_controller/"  + vehicleName + "/point_path", true),
	replanGoingUp(true)
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
	pointPathClient("/vehicle_controller/"  + vehicleName + "/point_path", true)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

bool PointPathSimActionExecutor::execute(std::shared_ptr<PointPathAction> action)
{
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->yoyo && (action->targetSlope >= M_PI / 2 || action->targetSlope <= 0))
	{
		return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		std::string velSub = "/vehicle_controller/" + vehicleName + "/command_target_velocity";
		if(!hasPublisher(velSub))
		{
			publishers.insert(std::make_pair(velSub, nh.advertise<vehicle_auto_control::Velocity>(velSub, 1000, true)));
		}

		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action->targetHorizontalVelocity;

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action->targetHorizontalVelocity * (sin(action->targetSlope) / cos(action->targetSlope));

		velMsg.rotationalVelocity = action->targetRotationalVelocity;
	
		auto publisher = publishers.find(velSub); 
		publisher->second.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	//Creates an action goal and sends it to the action server for point path movement
	pointPathGoal = vehicle_auto_control::PointPathGoal();

	for(unsigned int i = action->getCurrentPoint(); i < action->points.size(); i++)
	{
		auto point = action->points[i];
		geometry_msgs::Point p;
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		pointPathGoal.points.push_back(p);
	}

	//Set so the triggerReplan function knows when the yoyo direction changes
	replanGoingUp = true;

	pointPathGoal.upperDepth = action->upperDepth;
	pointPathGoal.lowerDepth = action->lowerDepth;
	pointPathGoal.yoyo = action->yoyo;

	pointPathClient.waitForServer();
	pointPathClient.sendGoal(pointPathGoal,
							 boost::bind(&PointPathSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&PointPathSimActionExecutor::actionActive, this, action),
							 boost::bind(&PointPathSimActionExecutor::actionFeedback, this, action, _1));
	return true;
}

void PointPathSimActionExecutor::cancel(std::shared_ptr<PointPathAction> action)
{
	pointPathClient.cancelAllGoals();
}

bool PointPathSimActionExecutor::triggerReplan(std::shared_ptr<PointPathAction> action)
{
	//trigger a replan when the top or bottom of a yoyo has been reached
	if(action->yoyo)
	{
		return yoyoTriggerReplan(action);
	}

	return flatTriggerReplan(action);
}

bool PointPathSimActionExecutor::yoyoTriggerReplan(std::shared_ptr<PointPathAction> action)
{
	//trigger a replan when the top or bottom of a yoyo has been reached
	if(action->getGoingUp() != replanGoingUp)
	{
		replanGoingUp = action->getGoingUp();
		return true;
	}

	return false;
}

bool PointPathSimActionExecutor::flatTriggerReplan(std::shared_ptr<PointPathAction> action)
{
	//trigger a replan when the top or bottom of a yoyo has been reached
	if(replanNextUpdate)
	{
		replanNextUpdate = false;
		return true;
	}
	return false;
}

void PointPathSimActionExecutor::actionDone(std::shared_ptr<PointPathAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::PointPathResultConstPtr& result)
{
	ROS_INFO("Planner: ActionDone Start");
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action->setState(Action::State::INTERRUPTED);
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
	}
	action->setCurrentPoint(result->totalPoints);
	ROS_INFO("Planner: ActionDone End");
}

void PointPathSimActionExecutor::actionActive(std::shared_ptr<PointPathAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void PointPathSimActionExecutor::actionFeedback(std::shared_ptr<PointPathAction> action,
					const vehicle_auto_control::PointPathFeedbackConstPtr& feedback)
{
	if(feedback->currentPoint != action->getCurrentPoint())
	{
		action->setCurrentPoint(feedback->currentPoint);
		action->addPointReachedTime(ros::Time::now());
		replanNextUpdate = true;
	}
	action->setGoingUp(feedback->goingUp);
}

bool PointPathSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}