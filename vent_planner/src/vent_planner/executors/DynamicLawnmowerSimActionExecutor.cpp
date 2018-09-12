#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "planner_framework/Action.h"
#include "data_server/GetPlumeData.h"
#include "vehicle_auto_control/Velocity.h"
#include "vent_planner/executors/DynamicLawnmowerSimActionExecutor.h"

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName, double loopHertz) :
	vehicleName(vehicleName),
	nh(nh),
	dynamicLawnmowerClient("planner/"  + vehicleName + "/dynamic_lawnmower", false),
	velPublisher(nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/" + vehicleName + "/command_target_velocity", 1000, true))
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("/vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(const DynamicLawnmowerSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh),
	dynamicLawnmowerClient("planner/"  + vehicleName + "/dynamic_lawnmower", false),
    velPublisher(nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/" + vehicleName + "/command_target_velocity", 1000, true))
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("/vehicles/get_info");
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
    ROS_INFO("Planner: Dynamic Lawnmower Action Execute");
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->targetSlope >= M_PI / 2 || action->targetSlope <= 0)
	{
		ROS_INFO("Planner: Dynamic Lawnmower Action Executor: Invalid target slope");
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
		ROS_INFO("Planner: Dynamic Lawnmower Action Executor: Unknown prop module");
		return false;
	}

	//Creates an action goal and sends it to the action server for point path movement
	dynamicLawnmowerGoal = vent_planner::DynamicLawnmowerRosGoal();

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
    ROS_INFO("Planner: Dynamic Lawnmower Send Goal");
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
    ROS_INFO("Planner: DynamicLawnmower Cancel");
	dynamicLawnmowerClient.cancelAllGoals();
}

bool DynamicLawnmowerSimActionExecutor::triggerReplan(std::shared_ptr<DynamicLawnmowerAction> action)
{
	bool replanReturn = replanNextUpdate;
	replanNextUpdate = false;
    if(replanReturn)
    {
        ROS_INFO("Planner: DynamicLawnmower Replan");
    }

	return false;
}

void DynamicLawnmowerSimActionExecutor::actionDone(std::shared_ptr<DynamicLawnmowerAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vent_planner::DynamicLawnmowerRosResultConstPtr& result)
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
					const vent_planner::DynamicLawnmowerRosFeedbackConstPtr& feedback)
{
	action->setCurrentTrack(feedback->currentTrack);
	action->setCurrentSection(feedback->currentSection);
}