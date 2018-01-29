#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "geometry_msgs/Point.h"

#include "vent_planner/VentActionExecutor.h"
#include "vent_planner/VentSimActionExecutor.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/YoYoPointPath.h"

VentSimActionExecutor::VentSimActionExecutor(ros::NodeHandle& nh) :
VentActionExecutor(nh)
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("vehicles/get_info");
	infoClient.waitForExistence();


	std::vector<std::string> vehicleNames;
	nh.getParam("vehicles/names", vehicleNames);

	for(std::string& name : vehicleNames)
	{
		underwater_vehicle_sim::GetVehicleInfo info;
		info.request.name = name;
		infoClient.call(info);
		vehicleInfo.insert(std::make_pair(name, info.response));
	}
}


void VentSimActionExecutor::executeYoYoPointPathAction(std::string vehicleName, 
													   double targetHorizontalVelocity, 
													   double targetRotationalVelocity,
													   double targetSlope, 
													   double minDepth,
													   double maxDepth,
													   std::vector<tf::Vector3>& points)
{
	auto info = vehicleInfo.find (vehicleName);

	//If there is no vehicle info for this vehicle name this action cannot be completed.
	if(info == vehicleInfo.end() || info->second.propModuleType == "")
	{
		return;
	}

	//targetSlope can only be on the interval (0, 90) degrees
	if(targetSlope >= M_PI / 2 || targetSlope <= 0)
	{
		return;
	}

	if(info->second.propModuleType == "FourDOFPropulsion")
	{
		std::string velSub = "/vehicle_controller/ " + vehicleName + "/command_target_velocity";
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
	
		publishers.find(velSub)->second.publish(velMsg);
	}
	else //If the prop module is not known then this cannot be completed
	{
		return;
	}

	//Publish message with points for the path
	std::string pathSub = "/vehicle_controller/ " + vehicleName + "/command_yoyo_point_path";
	if(!hasPublisher(pathSub))
	{
		publishers.insert(std::make_pair(pathSub, nh.advertise<vehicle_auto_control::YoYoPointPath>(pathSub, 1000)));
	}
	vehicle_auto_control::YoYoPointPath pathMsg;
	for(auto point : points)
	{
		geometry_msgs::Point p;
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
		pathMsg.points.push_back(p);
	}
	publishers.find(pathSub)->second.publish(pathMsg);
	
}

bool VentSimActionExecutor::hasPublisher(std::string topic)
{
	return publishers.find(topic) != publishers.end();
}