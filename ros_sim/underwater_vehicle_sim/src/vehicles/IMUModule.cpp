#include "vehicles/IMUModule.h"

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "sensor_msgs/Imu.h"

IMUModule::IMUModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName) :
	GeneralModule(name, "IMU", parentNH, vehicleName)
{
	nh.param("angular_velocity_variance", angularVelocityVariance, 0.00174533); //default is 0.1 deg in radians

	nh.param("heading_variance", headingVariance, 0.00872665); //default is 0.5 deg in radians
	nh.param("tilt_variance", tiltVariance, 0.0174533); //default is 1 deg in radians

	imu = nh.advertise<sensor_msgs::Imu>("data", 1000);
}

void IMUModule::update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	tf2::Quaternion rotation(vehicleState.getRotationNED());
	tf2::Vector3 angularVelocity = vehicleState.getAngularVelocity();

	//Add gaussian noise to the angular velocity
	for(unsigned int i = 0; i < 3; i++)
	{
		std::normal_distribution<double> rotationDistribution(0, sqrt(angularVelocityVariance));
		angularVelocity[i] += rotationDistribution(generator);
	}

	std::normal_distribution<double> headingDist(0, sqrt(headingVariance));
	std::normal_distribution<double> tiltDist(0, sqrt(tiltVariance));

	
	tf2::Quaternion rotationError;
	rotationError.setRPY(tiltDist(generator), tiltDist(generator), headingDist(generator));
	rotation = rotation * rotationError;

	sensor_msgs::Imu data;

	data.header.stamp = lastTime;
  	data.header.frame_id = "world_ned";

	data.orientation.x = rotation.x();
	data.orientation.y = rotation.y();
	data.orientation.z = rotation.z();
	data.orientation.w = rotation.w();

	data.angular_velocity.x = angularVelocity.x();
	data.angular_velocity.y = angularVelocity.y();
	data.angular_velocity.z = angularVelocity.z();

	data.orientation_covariance[0] = tiltVariance * tiltVariance;
	data.orientation_covariance[1] = 0;
	data.orientation_covariance[2] = 0;

	data.orientation_covariance[3] = 0;
	data.orientation_covariance[4] = tiltVariance * tiltVariance;
	data.orientation_covariance[5] = 0;

	data.orientation_covariance[6] = 0;
	data.orientation_covariance[7] = 0;
	data.orientation_covariance[8] = headingVariance * headingVariance;


	data.angular_velocity_covariance[0] = angularVelocityVariance * angularVelocityVariance;
	data.angular_velocity_covariance[1] = 0;
	data.angular_velocity_covariance[2] = 0;

	data.angular_velocity_covariance[3] = 0;
	data.angular_velocity_covariance[4] = angularVelocityVariance * angularVelocityVariance;
	data.angular_velocity_covariance[5] = 0;

	data.angular_velocity_covariance[6] = 0;
	data.angular_velocity_covariance[7] = 0;
	data.angular_velocity_covariance[8] = angularVelocityVariance * angularVelocityVariance;


	data.linear_acceleration_covariance[0] = -1;

	data.orientation_covariance[0];
	imu.publish(data);
}