#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"

#include "vent_planner/DynamicLawnmowerSimActionExecutor.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/DynamicLawnmowerAction.h"

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh),
	dynamicLawnmowerClient("/vehicle_controller/"  + vehicleName + "/dynamic_lawnmower", true),
	velPublisher(nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/" + vehicleName + "/command_target_velocity", 1000, true))
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(const DynamicLawnmowerSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh),
	dynamicLawnmowerClient("/vehicle_controller/"  + vehicleName + "/dynamic_lawnmower", true)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> DynamicLawnmowerSimActionExecutor::clone()
{
	std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> a(new DynamicLawnmowerSimActionExecutor(*this));
    return a;
}


bool DynamicLawnmowerSimActionExecutor::execute(std::shared_ptr<DynamicLawnmowerAction> action)
{


	//targetSlope can only be on the interval (0, 90) degrees
	if(action->targetSlope >= M_PI / 2 || action->targetSlope <= 0)
	{
		return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action->targetHorizontalVelocity;

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action->targetHorizontalVelocity * (sin(action->targetSlope) / cos(action->targetSlope));

		velMsg.rotationalVelocity = action->targetRotationalVelocity;
	
		velPublisher.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	//Creates an action goal and sends it to the action server for point path movement
	dynamicLawnmowerGoal = vehicle_auto_control::DynamicLawnmowerGoal();

	dynamicLawnmowerGoal.startLocation.x = action->startLocation.getX();
	dynamicLawnmowerGoal.startLocation.y = action->startLocation.getY();
	dynamicLawnmowerGoal.startLocation.z = action->startLocation.getZ();

	dynamicLawnmowerGoal.currentTrack = action->getCurrentTrack();
	dynamicLawnmowerGoal.currentSection = action->getCurrentSection();
	dynamicLawnmowerGoal.alongTrackDirection = action->alongTrackDirection;
	dynamicLawnmowerGoal.acrossTrackDirection = action->acrossTrackDirection;
	dynamicLawnmowerGoal.trackSpacing = action->trackSpacing;
	dynamicLawnmowerGoal.targetHeight = action->targetHeight;

	dynamicLawnmowerGoal.minSectionsPerTrack = action->minSectionsPerTrack;
	dynamicLawnmowerGoal.continueThreshold = action->continueThreshold;
	dynamicLawnmowerGoal.trackSectionThreshold = action->trackSectionThreshold;

	dynamicLawnmowerClient.waitForServer();
	dynamicLawnmowerClient.sendGoal(dynamicLawnmowerGoal,
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionActive, this, action),
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionFeedback, this, action, _1));

	//replan at the start of each action, i.e. the end of the previous action
	replanNextUpdate = true;
	
	return true;
}

void DynamicLawnmowerSimActionExecutor::cancel(std::shared_ptr<DynamicLawnmowerAction> action)
{
	dynamicLawnmowerClient.cancelAllGoals();
}

bool DynamicLawnmowerSimActionExecutor::triggerReplan(std::shared_ptr<DynamicLawnmowerAction> action)
{
	bool replanReturn = replanNextUpdate;
	replanNextUpdate = false;
	return replanReturn;
}

void DynamicLawnmowerSimActionExecutor::actionDone(std::shared_ptr<DynamicLawnmowerAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::DynamicLawnmowerResultConstPtr& result)
{
	ROS_INFO("Planner: DynamicLawnmower ActionDone Start");
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

	ROS_INFO("Planner: DynamicLawnmower ActionDone End");
}

void DynamicLawnmowerSimActionExecutor::actionActive(std::shared_ptr<DynamicLawnmowerAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void DynamicLawnmowerSimActionExecutor::actionFeedback(std::shared_ptr<DynamicLawnmowerAction> action,
					const vehicle_auto_control::DynamicLawnmowerFeedbackConstPtr& feedback)
{
	action->setCurrentTrack(feedback->currentTrack);
	action->setCurrentSection(feedback->currentSection);
}