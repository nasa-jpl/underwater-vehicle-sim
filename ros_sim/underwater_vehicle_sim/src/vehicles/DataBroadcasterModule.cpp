#include "ros/ros.h"

#include "model_server/GetModelData.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "underwater_vehicle_msgs/VehicleData.h"
#include "vehicles/DataBroadcasterModule.h"

using namespace ocean_models;

#define SECONDS_IN_DAY 86400

DataBroadcasterModule::DataBroadcasterModule(std::string name) :
	GeneralModule(name, "DataBroadcaster")
{
	dataRecorder = nh.advertise<underwater_vehicle_msgs::VehicleData>("data", 1000);
	client = nh.serviceClient<model_server::GetModelData>("/get_model_data");
}

void DataBroadcasterModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	if(!(std::isnan(modelData.u) &&
	     std::isnan(modelData.v) &&
	     std::isnan(modelData.temp) &&
	     std::isnan(modelData.salt) &&
	     std::isnan(modelData.dye) &&
	     std::isnan(modelData.depth)))
	{
		tf2::Vector3 nedPosition = vehicleState.getPositionNED();
		tf2::Vector3 enuPosition = vehicleState.getPositionENU();

		underwater_vehicle_msgs::VehicleDataPtr data(new underwater_vehicle_msgs::VehicleData);

		data->name = name;
		data->x = nedPosition.getX();
		data->y = nedPosition.getY();
		data->h = nedPosition.getZ();
		data->time = lastTime;

		float precisionPow = std::pow(10, 4); //Set precision of temperature reading to 4 decimal places
		data->temp = std::round(modelData.temp * precisionPow) / precisionPow;
		
		data->salt = modelData.salt;
		data->dye = modelData.dye;
		data->sonarDepth = modelData.depth - nedPosition.getZ();

		dataRecorder.publish(data);
	}
}