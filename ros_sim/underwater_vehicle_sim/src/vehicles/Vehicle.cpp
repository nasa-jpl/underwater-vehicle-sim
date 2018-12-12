#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "vehicles/DataBroadcasterModule.h"

#include "vehicles/PowerCapacityModule.h"
#include "vehicles/DataCapacityModule.h"
#include "vehicles/FourDOFPropulsion.h"

#include "underwater_vehicle_msgs/VehicleData.h"



Vehicle::Vehicle(std::string name, ros::NodeHandle& parentNH) :
	name(name),
	nh(ros::NodeHandle(parentNH, "vehicles/" + name)),
	vehicleState(parentNH)
{
	initalizeVehicleFrame();
  	
	initalizePropulsionModule();
	initalizeGeneralModules();
}

Vehicle::Vehicle(Vehicle&& other)
	: propulsionModule(std::move(other.propulsionModule)), 
      modules(std::move(other.modules)),
      name(std::move(other.name)),
      nh(std::move(other.nh)),
      lastTransformTime(std::move(other.lastTransformTime)),
	  vehicleState(std::move(other.vehicleState))
{}

void Vehicle::initalizeVehicleFrame()
{
	//Get the parameters for the starting location of the vehicle
	float startX = 0;
	float startY = 0;
	float startZ = 0;
	
	nh.getParam("start_x", startX);
	nh.getParam("start_y", startY);
	nh.getParam("start_z", startZ);
	nh.getParam("start_power", powerCapacity);
	nh.getParam("start_dataCapacity", dataCapacity);

	//broadcast the inital frame for this vehicle

	tf2::Quaternion initialRotation;
	initialRotation.setRPY(0, 0, 0);
	vehicleState.setRotationENU(initialRotation);

	tf2::Vector3 initialPosition(startX, startY, startZ);
	vehicleState.setPositionENU(initialPosition);
  	broadcastTransform();
  	
}

void Vehicle::initalizePropulsionModule()
{
	std::string propModuleName;

	//get the name of the propulsion module and create the needed 
	if(nh.hasParam("propModuleName"))
	{

		nh.getParam("propModuleName", propModuleName);
		propulsionModule = PropulsionModule::makePropulsionModule(propModuleName, vehicleState, nh);
	}
}

void Vehicle::initalizeGeneralModules()
{
	std::vector<std::string> moduleNames;
	nh.getParam("moduleNames", moduleNames);

	for(std::string& name : moduleNames)
	{
		modules.push_back(GeneralModule::makeGeneralModule(name, nh, getName()));
	}
}

void Vehicle::update()
{
	ros::Time currentTime = ros::Time::now();
	vehicleState.updatePose(currentTime, currentTime - lastTransformTime);
	lastTransformTime = currentTime;

	broadcastTransform();

	//update all modules
	for(std::unique_ptr<GeneralModule>& module : modules)
	{
		module->updateAtRate(name, lastTransformTime, vehicleState);
	}
}

void Vehicle::getInfo(underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
    if(propulsionModule)
    {
        res.propModuleName = propulsionModule->getName();
        res.propModuleType = propulsionModule->getType();  
    }
    else
    {
        res.propModuleName = "";
        res.propModuleType = "";
    }
	
	for(std::unique_ptr<GeneralModule>& module : modules)
	{
		res.moduleNames.push_back(module->getName());
		res.moduleTypes.push_back(module->getType());
	}
}

void Vehicle::broadcastTransform()
{
	static tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = lastTransformTime;
  	transformStamped.header.frame_id = "world_ned";
  	transformStamped.child_frame_id = name;

	tf2::Vector3 position = vehicleState.getPositionNED();
	transformStamped.transform.translation.x = position.x();
	transformStamped.transform.translation.y = position.y();
	transformStamped.transform.translation.z = position.z();

	tf2::Quaternion rotation = vehicleState.getRotationNED();
	transformStamped.transform.rotation.x = rotation.x();
	transformStamped.transform.rotation.y = rotation.y();
	transformStamped.transform.rotation.z = rotation.z();
	transformStamped.transform.rotation.w = rotation.w();
	
  	br.sendTransform(transformStamped);
}

std::string Vehicle::getName()
{
	return name;
}
