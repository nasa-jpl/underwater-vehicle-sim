#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

#include <random>

class IMUModule : public GeneralModule
{

public:
	IMUModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName);
	~IMUModule() {}

	void update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData);

private:
	
private:
	ros::Publisher imu;
	double angularVelocityVariance;
	double headingVariance;
	double tiltVariance;

	std::default_random_engine generator;
};

#endif
