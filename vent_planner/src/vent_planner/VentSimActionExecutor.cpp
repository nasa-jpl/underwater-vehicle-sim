#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"
#include "vent_planner/VentActionExecutor.h"
#include "vent_planner/VentSimActionExecutor.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"

VentSimActionExecutor::VentSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName) :
VentActionExecutor(nh, vehicleName),
pointPathClient("/vehicle_controller/"  + vehicleName + "/point_path", true)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}


void VentSimActionExecutor::executeYoYoPointPathAction(double targetHorizontalVelocity, 
													   double targetRotationalVelocity,
													   double targetSlope, 
													   double upperDepth,
													   double lowerDepth,
													   std::vector<tf::Vector3>& points)
{
	//targetSlope can only be on the interval (0, 90) degrees
	if(targetSlope >= M_PI / 2 || targetSlope <= 0)
	{
		return;
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
		velMsg.horizontalVelocity = targetHorizontalVelocity;

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = targetHorizontalVelocity * (sin(targetSlope) / cos(targetSlope));
		velMsg.rotationalVelocity = targetRotationalVelocity;
	
		auto publisher = publishers.find(velSub); 
		publisher->second.publish(velMsg);
	}
	else //If the prop module is not known then this cannot be completed
	{
		return;
	}

	pointPathGoal = vehicle_auto_control::PointPathGoal();

	for(auto point : points)
	{
		geometry_msgs::Point p;
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		pointPathGoal.points.push_back(p);
	}
	pointPathGoal.upperDepth = upperDepth;
	pointPathGoal.lowerDepth = lowerDepth;
	pointPathGoal.yoyo = true;

	pointPathClient.waitForServer();
	pointPathClient.sendGoal(pointPathGoal);
}

void VentSimActionExecutor::monitorYoYoPointPathAction(Action::State& state)
{
	if(pointPathClient.getState() == actionlib::SimpleClientGoalState::RECALLED ||
	   pointPathClient.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		state = Action::State::INTERRUPTED;
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::REJECTED ||
			pointPathClient.getState() == actionlib::SimpleClientGoalState::ABORTED ||
			pointPathClient.getState() == actionlib::SimpleClientGoalState::LOST)
	{
		state = Action::State::FAILED;
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::ACTIVE)
	{
		state = Action::State::EXECUTING;
	}
	else if(pointPathClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		state = Action::State::COMPLETED;
	}
}

bool VentSimActionExecutor::triggerReplanYoYoPointPathAction()
{
	return false;
}

bool VentSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}