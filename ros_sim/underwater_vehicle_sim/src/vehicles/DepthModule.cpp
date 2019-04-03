#include "vehicles/DepthModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "underwater_vehicle_msgs/FloatMeasurement.h"

DepthModule::DepthModule(std::string name) :
	GeneralModule(name, "Depth")
{
	ros::NodeHandle nhPriv("~/" + name);
	nhPriv.param("depth_random_error", depthRandomError, 0.0);
	nhPriv.param("depth_bias_error", depthBiasError, 0.0);
	
	int randomSeed;
	if(nhPriv.getParam("random_seed", randomSeed))
	{
		generator.seed(randomSeed);
	}

	depthDistribution = std::normal_distribution<double>(depthBiasError, sqrt(depthRandomError));

	depth = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("data", 1000);
}

void DepthModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	double depthReading = vehicleState.getPositionNED().getZ();
	
	//Only apply error if the std dev of the distribution is positive
	if(depthRandomError > 0)
	{
		depthReading += depthDistribution(generator);
	}

	underwater_vehicle_msgs::FloatMeasurementPtr depthMsg(new underwater_vehicle_msgs::FloatMeasurement);

	depthMsg->header.frame_id = "world_ned";
	depthMsg->header.stamp = lastTime;
	depthMsg->data = depthReading;
	depthMsg->variance = depthRandomError;

	depth.publish(depthMsg);
}