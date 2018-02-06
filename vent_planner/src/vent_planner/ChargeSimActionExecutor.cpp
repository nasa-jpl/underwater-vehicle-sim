#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "planner_framework/Action.h"

#include "vent_planner/ChargeSimActionExecutor.h"
#include "vent_planner/actions/ChargeAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"

ChargeSimActionExecutor::ChargeSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

ChargeSimActionExecutor::ChargeSimActionExecutor(const ChargeSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

bool ChargeSimActionExecutor::execute(std::shared_ptr<ChargeAction> action)
{
	return true;
}

void ChargeSimActionExecutor::cancel(std::shared_ptr<ChargeAction> action)
{
}

bool ChargeSimActionExecutor::triggerReplan(std::shared_ptr<ChargeAction> action)
{
	return false;
}

void ChargeSimActionExecutor::actionDone(std::shared_ptr<ChargeAction> action,
					const actionlib::SimpleClientGoalState& state)
{

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
}

void ChargeSimActionExecutor::actionActive(std::shared_ptr<ChargeAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void ChargeSimActionExecutor::actionFeedback(std::shared_ptr<ChargeAction> action)
{
}

bool ChargeSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}
