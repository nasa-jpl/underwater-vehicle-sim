#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "vehicles/GeneralModule.h"
#include "vehicles/PropulsionModule.h"

#include "vehicles/DataBroadcasterModule.h"

#include "vehicles/FourDOFPropulsion.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#define SECONDS_IN_DAY 86400

using namespace ocean_models;

Vehicle::Vehicle()
{
	ros::NodeHandle nhPriv("~");
	nhPriv.getParam("evect_by_currents", evectByCurrents);

	modelClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");

	initalizeVehicleFrame();
  	
	initalizePropulsionModule();
	initalizeGeneralModules();
}

Vehicle::Vehicle(std::unique_ptr<ModelInterface> model)	:
	model(std::move(model))
{
	ros::NodeHandle nhPriv("~");
	nhPriv.getParam("evect_by_currents", evectByCurrents);

	if(model == NULL)
	{
		modelClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");
	}

	initalizeVehicleFrame();
  	
	initalizePropulsionModule();
	initalizeGeneralModules();
}

Vehicle::Vehicle(Vehicle&& other)
	: propulsionModule(std::move(other.propulsionModule)), 
      modules(std::move(other.modules)),
      nh(std::move(other.nh)),
      lastTransformTime(std::move(other.lastTransformTime)),
	  vehicleState(std::move(other.vehicleState)),
	  evectByCurrents(other.evectByCurrents),
	  model(std::move(other.model))
{}

void Vehicle::initalizeVehicleFrame()
{
	ros::NodeHandle nhPriv("~");
	//Get the parameters for the starting location of the vehicle
	nhPriv.getParam("start_x", startX);
	nhPriv.getParam("start_y", startY);
	nhPriv.getParam("start_z", startZ);
	nhPriv.getParam("start_power", powerCapacity);
	nhPriv.getParam("start_dataCapacity", dataCapacity);

	//broadcast the inital frame for this vehicle
	tf2::Quaternion initialRotation;
	initialRotation.setRPY(0, 0, 0);
	vehicleState.setRotationNED(initialRotation);

	tf2::Vector3 initialPosition(startX, startY, startZ);
	vehicleState.setPositionNED(initialPosition);

	//Set transform time and data
	lastTransformTime = ros::Time::now();
	dataAtLastTransform = getModelData();

	broadcastTransform();
}

void Vehicle::initalizePropulsionModule()
{
	ros::NodeHandle nhPriv("~");
	double propulsionHertz = 1;
	nhPriv.getParam("propulsion_hertz", propulsionHertz);

	propulsionModule = PropulsionModule::makePropulsionModule(vehicleState);

	propTimer = nh.createTimer(ros::Duration(1 / propulsionHertz), std::bind(&Vehicle::propModuleTimerCallback, this));
	propTimer.start();
}

void Vehicle::initalizeGeneralModules()
{
	ros::NodeHandle nhPriv("~");
	std::vector<std::string> moduleNames;
	nhPriv.getParam("moduleNames", moduleNames);

	for(std::string& name : moduleNames)
	{
		double moduleHertz = 1;
		nhPriv.getParam(name + "/hertz", moduleHertz);

		modules.push_back(GeneralModule::makeGeneralModule(name));
		moduleTimers.push_back(nh.createTimer(ros::Duration(1 / moduleHertz),
											  std::bind(&Vehicle::moduleTimerCallback, this, modules.size() - 1)));
		moduleTimers[moduleTimers.size() - 1].start();
	}
}

void Vehicle::propModuleTimerCallback()
{
	propulsionModule->update();
}

void Vehicle::moduleTimerCallback(unsigned int moduleIndex)
{
	modules[moduleIndex]->update(lastTransformTime, vehicleState, dataAtLastTransform);
}

void Vehicle::update()
{
	ros::Time currentTime = ros::Time::now();
	ros::Duration deltaTime = currentTime - lastTransformTime;

	//Update the transform
	vehicleState.updatePose(currentTime, deltaTime);
	if(evectByCurrents)
	{
		vehicleState.evectByCurrents(dataAtLastTransform, deltaTime);
	}

	//Update time and data
	lastTransformTime = currentTime;
	dataAtLastTransform = getModelData();

	//Prevent the vehicle from clipping through the seafloor
	if(vehicleState.seafloorCollision(dataAtLastTransform))
	{
		dataAtLastTransform = getModelData();
	}

	//Broadcast the latest transform
	broadcastTransform();
}

ModelData Vehicle::getModelData()
{
	ModelData data;
	tf2::Vector3 enuPosition = vehicleState.getPositionENU();

	if(model != NULL)
	{
		try
		{
			data = model->getData(enuPosition.getX(), 
								  enuPosition.getY(),
								  enuPosition.getZ(), 
								  lastTransformTime.toSec() / SECONDS_IN_DAY);
		}
		catch(const std::out_of_range& e)
		{
			ROS_INFO("ModelServer: Out of Range: %f %f %f %f", enuPosition.getX(), 
															   enuPosition.getY(), 
															   enuPosition.getZ(), 
															   lastTransformTime.toSec() / SECONDS_IN_DAY);
			data = model->getDataOutOfRange(enuPosition.getX(), 
											enuPosition.getY(), 
											enuPosition.getZ(), 
											lastTransformTime.toSec() / SECONDS_IN_DAY);
		}    
	}
	else
	{
		model_server::GetModelData srv;

		srv.request.x = enuPosition.getX();
		srv.request.y = enuPosition.getY();
		srv.request.h = enuPosition.getZ();
		srv.request.time = lastTransformTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days


		bool success = modelClient.call(srv);

		if(success)
		{
			data.u = srv.response.u;
			data.v = srv.response.v;
			data.temp = srv.response.temp;
			data.salt = srv.response.salt;
			data.dye = srv.response.dye;
			data.depth = srv.response.depth;
		}
		else
		{
			data.u = std::numeric_limits<double>::quiet_NaN();
			data.v = std::numeric_limits<double>::quiet_NaN();
			data.temp = std::numeric_limits<double>::quiet_NaN();
			data.salt = std::numeric_limits<double>::quiet_NaN();
			data.dye = std::numeric_limits<double>::quiet_NaN();
			data.depth = std::numeric_limits<double>::quiet_NaN();
		}
	}

	

	return data;
}

void Vehicle::getInfo(underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
    if(propulsionModule)
    {
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

	res.startX = startX;
	res.startY = startY;
	res.startZ = startZ;
}

void Vehicle::broadcastTransform()
{
	static tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = lastTransformTime;
  	transformStamped.header.frame_id = "world_ned";
  	transformStamped.child_frame_id = nh.getNamespace();

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