#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "planner_framework/Action.h"
#include "data_server/GetPlumeData.h"
#include "vehicle_auto_control/Velocity.h"
#include "vent_planner/executors/DynamicLawnmowerSimActionExecutor.h"

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
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
    ROS_DEBUG("Execute dynamic lawnmower action");
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->getTargetSlope() >= M_PI / 2 || action->getTargetSlope() <= 0)
	{
		ROS_WARN("Dynamic lawnmower action has invalid target slope");
	   return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action->getTargetHorizontalVelocity();

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action->getTargetHorizontalVelocity() * (sin(action->getTargetSlope()) / cos(action->getTargetSlope()));

		velMsg.rotationalVelocity = action->getTargetRotationalVelocity();
	
		velPublisher.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		ROS_WARN("%s has unknown prop module. Cannot execute dynamic lawnmower", vehicleName.c_str());
		return false;
	}

	//Creates an action goal and sends it to the action server for point path movement
	dynamicLawnmowerGoal = vent_planner::DynamicLawnmowerRosGoal();

	dynamicLawnmowerGoal.startLocation.x = action->getStartLocation().getX();
	dynamicLawnmowerGoal.startLocation.y = action->getStartLocation().getY();
	dynamicLawnmowerGoal.startLocation.z = action->getStartLocation().getZ();

	dynamicLawnmowerGoal.currentTrack = action->getCurrentTrack();
	dynamicLawnmowerGoal.currentSection = action->getCurrentSection();
	dynamicLawnmowerGoal.alongTrackDirection = action->getAlongTrackDirection();
	dynamicLawnmowerGoal.acrossTrackDirection = action->getAcrossTrackDirection();
	dynamicLawnmowerGoal.trackSpacing = action->getTrackSpacing();
	dynamicLawnmowerGoal.targetHeight = action->getTargetHeight();

	dynamicLawnmowerGoal.minSectionsPerTrack = action->getMinSectionsPerTrack();
	dynamicLawnmowerGoal.continueThreshold = action->getContinueThreshold();
	dynamicLawnmowerGoal.trackSectionThreshold = action->getTrackSectionThreshold();

	dynamicLawnmowerClient.waitForServer();
    ROS_INFO("Send goal to dynamic lawnmower action server");
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
    ROS_INFO("Cancel dynamic lawnmower action");
	dynamicLawnmowerClient.cancelAllGoals();
}

bool DynamicLawnmowerSimActionExecutor::triggerReplan(std::shared_ptr<DynamicLawnmowerAction> action)
{
	bool replanReturn = replanNextUpdate;
	replanNextUpdate = false;
    if(replanReturn)
    {
        ROS_INFO("Replan during dynamic lawnmower action");
    }

	return false;
}

void DynamicLawnmowerSimActionExecutor::actionDone(std::shared_ptr<DynamicLawnmowerAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vent_planner::DynamicLawnmowerRosResultConstPtr& result)
{
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action->setState(Action::State::INTERRUPTED);
		ROS_INFO("Dynamic lawnmower action interrupted");
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
		ROS_INFO("Dynamic lawnmower action failed");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
		ROS_INFO("Dynamic lawnmower action completed");
	}
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