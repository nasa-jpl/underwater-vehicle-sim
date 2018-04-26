#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "vehicles/DataBroadcasterModule.h"

#define SECONDS_IN_DAY 86400

DataBroadcasterModule::DataBroadcasterModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "DataBroadcaster", parentNH, vehicleName)
{

	dataRecorder = nh.advertise<underwater_vehicle_sim::VehicleData>("data", 1000);
	client = nh.serviceClient<model_server::GetModelData>("get_model_data");
}

void DataBroadcasterModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	model_server::GetModelData srv;

	srv.request.x = position.getX();
	srv.request.y = position.getY();
	srv.request.h = position.getZ();
	srv.request.time = lastTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days

	if(client.exists())
	{
		bool success = client.call(srv);

		if(success)
		{
			underwater_vehicle_sim::VehicleData data;

			data.name = name;
			data.x = position.getX();
			data.y = position.getY();
			data.h = position.getZ();
			data.time = lastTime;

			float precisionPow = std::pow(10, 4); //Set presision of temperature reading to 4 decimal places
			data.temp = std::round(srv.response.temp * precisionPow) / precisionPow;
			
			data.salt = srv.response.salt;
			data.dye = srv.response.dye;
			data.sonarDepth = srv.response.depth + position.getZ(); //depth + z, becuase z is negative while depth is positive

			dataRecorder.publish(data);

			//sizeof gives the size of data to be 64 Bytes
			dataCapacity -= 64;
		}
	}
}