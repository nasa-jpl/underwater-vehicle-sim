#include "ros/ros.h"

#include "model_server/GetModelData.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "underwater_vehicle_msgs/VehicleData.h"
#include "vehicles/DataBroadcasterModule.h"

#define SECONDS_IN_DAY 86400

DataBroadcasterModule::DataBroadcasterModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "DataBroadcaster", parentNH, vehicleName)
{

	dataRecorder = nh.advertise<underwater_vehicle_msgs::VehicleData>("data", 1000);
	client = nh.serviceClient<model_server::GetModelData>("/get_model_data");
}

void DataBroadcasterModule::update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState) 
{
	model_server::GetModelData srv;

	tf2::Vector3 position = vehicleState.getPositionENU();
	srv.request.x = position.getX();
	srv.request.y = position.getY();
	srv.request.h = position.getZ();
	srv.request.time = lastTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days

	if(client.exists())
	{
		bool success = client.call(srv);

		if(success)
		{
			underwater_vehicle_msgs::VehicleDataPtr data(new underwater_vehicle_msgs::VehicleData);

			data->name = name;
			data->x = position.getX();
			data->y = position.getY();
			data->h = position.getZ();
			data->time = lastTime;

			float precisionPow = std::pow(10, 4); //Set presision of temperature reading to 4 decimal places
			data->temp = std::round(srv.response.temp * precisionPow) / precisionPow;
			
			data->salt = srv.response.salt;
			data->dye = srv.response.dye;
			data->sonarDepth = srv.response.depth + position.getZ(); //depth + z, becuase z is negative while depth is positive

			dataRecorder.publish(data);
		}
	}
}