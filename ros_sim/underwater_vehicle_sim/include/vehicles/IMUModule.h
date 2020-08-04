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
	IMUModule(std::string name);
	~IMUModule() {}

	/**
	* The vehicle orientation is taken and noise is added. The noise characteristics in the yaw direction
	* can be set independently that in the roll and pitch direction. No seperate field in the resulting message
	* contains magnetic heading information. This information can be pulled from the orientation.
	*/
	void update(const ros::Time& lastTime, VehicleState& vehicleState, ocean_models::ModelData& modelData);

private:
	
private:
	ros::Publisher imu;

	double angularVelocityStdDev;
	std::vector<double> angularVelocityBiasError;
	bool angVelActive;

	double headingStdDev;
	double headingBiasError;
	bool headingActive;

	double rollPitchStdDev;
	double rollBiasError;
	double pitchBiasError;

	std::default_random_engine generator;
	std::normal_distribution<double> rotationDistribution;
	std::normal_distribution<double> headingDist;
	std::normal_distribution<double> tiltDist;

};

#endif
