#include "ros/ros.h"
#include <iostream>

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "vehicles/DataRecorderModule.h"

#define SECONDS_IN_DAY 86400

DataRecorderModule::DataRecorderModule(std::string name, ros::NodeHandle& parentNH) :
	GeneralModule(name, "DataRecorder", parentNH)
{

	dataRecorder = nh.advertise<underwater_vehicle_sim::VehicleData>("/data_server/put", 1000);
	client = nh.serviceClient<model_server::GetModelData>("/get_model_data");
}

void DataRecorderModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	model_server::GetModelData srv;

	srv.request.x = position.getX();
	srv.request.y = position.getY();
	srv.request.h = position.getZ();
	srv.request.time = lastTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days


	if(client.exists())
	{
		client.call(srv);

		underwater_vehicle_sim::VehicleData data;

		data.name = name;
		data.x = position.getX();
		data.y = position.getY();
		data.h = position.getZ();
		data.time = lastTime;
		data.temp = srv.response.temp;
		data.salt = srv.response.salt;
		data.dye = srv.response.dye;

		dataRecorder.publish(data);

		//sizeof gives the size of data to be 64 Bytes
		dataCapacity -= 64;
	}
}
