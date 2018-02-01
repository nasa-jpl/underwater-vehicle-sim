#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"

#include "vent_planner/YoYoPointPathSimActionExecutor.h"
#include "vent_planner/actions/YoYoPointPathAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"

YoYoPointPathSimActionExecutor::YoYoPointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
	vehicleName(vehicleName),
	nh(nh),
	pointPathClient("/vehicle_controller/"  + vehicleName + "/point_path", true)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

YoYoPointPathSimActionExecutor::YoYoPointPathSimActionExecutor(const YoYoPointPathSimActionExecutor& other) :
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

bool YoYoPointPathSimActionExecutor::execute(YoYoPointPathAction& action)
{
	//targetSlope can only be on the interval (0, 90) degrees
	if(action.targetSlope >= M_PI / 2 || action.targetSlope <= 0)
	{
		return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		std::string velSub = "/vehicle_controller/" + vehicleName + "/command_target_velocity";
		if(!hasPublisher(velSub))
		{
			publishers.insert(std::make_pair(velSub, nh.advertise<vehicle_auto_control::Velocity>(velSub, 1000)));
		}

		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action.targetHorizontalVelocity;

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action.targetHorizontalVelocity * (sin(action.targetSlope) / cos(action.targetSlope));
		velMsg.rotationalVelocity = action.targetRotationalVelocity;
	
		auto publisher = publishers.find(velSub); 
		publisher->second.publish(velMsg);
	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}

	//Creates an action goal and sends it to the action server for point path movement
	pointPathGoal = vehicle_auto_control::PointPathGoal();

	for(auto point : action.points)
	{
		geometry_msgs::Point p;
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		pointPathGoal.points.push_back(p);
	}
	pointPathGoal.upperDepth = action.upperDepth;
	pointPathGoal.lowerDepth = action.lowerDepth;
	pointPathGoal.yoyo = true;

	pointPathClient.waitForServer();
	pointPathClient.sendGoal(pointPathGoal);
	return true;
}

void YoYoPointPathSimActionExecutor::monitor(YoYoPointPathAction& action)
{
	//Determines the state of the action based on the state of the goal in the action server
	if(pointPathClient.getState() == actionlib::SimpleClientGoalState::RECALLED ||
	   pointPathClient.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action.setState(Action::State::INTERRUPTED);
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::REJECTED ||
			pointPathClient.getState() == actionlib::SimpleClientGoalState::ABORTED ||
			pointPathClient.getState() == actionlib::SimpleClientGoalState::LOST)
	{
		action.setState(Action::State::FAILED);
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::ACTIVE)
	{
		action.setState(Action::State::EXECUTING);
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action.setState(Action::State::COMPLETED);
	}
}

bool YoYoPointPathSimActionExecutor::triggerReplan(YoYoPointPathAction& action)
{
	return false;
}

bool YoYoPointPathSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}