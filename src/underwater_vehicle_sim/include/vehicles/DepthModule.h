#ifndef DEPTH_MODULE_H
#define DEPTH_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

#include <random>

class DepthModule : public GeneralModule
{

public:
	DepthModule(std::string name);
	~DepthModule() {}

	void update(const ros::Time& lastTime, VehicleState& vehicleState, ocean_model_interfaces::ModelData& modelData);

private:
	
private:
	ros::Publisher depth;

	/**
	*Error in depth measurment
	*/
	double depthStdDev;
	double depthBiasError;
	
	std::default_random_engine generator;
	std::normal_distribution<double> depthDistribution;
};

#endif
