#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "vehicles/DataRecorderModule.h"

DataRecorderModule::DataRecorderModule(std::string name, ros::NodeHandle& parentNH) :
	GeneralModule(name, parentNH)
{
	dataRecorder = nh.advertise<underwater_vehicle_sim::VehicleData>("/data_server/put", 1000);
	client = nh.serviceClient<model_server::GetModelData>("/get_model_data");
}

void DataRecorderModule::update(const ros::Time& lastTime, const tf::Vector3& position) 
{
	model_server::GetModelData srv;

	srv.request.x = position.getX();
	srv.request.y = position.getY();
	srv.request.h = position.getZ();
	srv.request.time = lastTime.toSec() / 86400; //convert from seconds to days


	client.call(srv);

	underwater_vehicle_sim::VehicleData data;

	data.x = position.getX();
	data.y = position.getY();
	data.h = position.getZ();
	data.time = lastTime.toSec() / 86400;
	data.temp = srv.response.temp;
	data.salt = srv.response.salt;
	data.dye = srv.response.dye;

	dataRecorder.publish(data);
}




