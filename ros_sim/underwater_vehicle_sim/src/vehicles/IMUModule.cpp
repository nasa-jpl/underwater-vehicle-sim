#include "vehicles/IMUModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "sensor_msgs/Imu.h"

using namespace ocean_models;

IMUModule::IMUModule(std::string name) :
	GeneralModule(name, "IMU")
{
	ros::NodeHandle nhPriv("~/" + name);

	nhPriv.param("angular_velocity_random_error", angularVelocityStdDev, 0.0);
	nhPriv.param("angular_velocity_bias_error", angularVelocityBiasError, std::vector<double>{0.0, 0.0, 0.0});
	nhPriv.param("angular_velocity_active", angVelActive, true);

	nhPriv.param("heading_random_error", headingStdDev, 0.0);
	nhPriv.param("heading_bias_error", headingBiasError, 0.0);
	nhPriv.param("heading_active", headingActive, true);

	nhPriv.param("roll_pitch_random_error", rollPitchStdDev, 0.0);
	nhPriv.param("roll_bias_error", rollBiasError, 0.0); 
	nhPriv.param("pitch_bias_error", pitchBiasError, 0.0);

	int randomSeed;
	if(nhPriv.getParam("random_seed", randomSeed))
	{
		generator = std::default_random_engine(randomSeed);
	}

	headingDist = std::normal_distribution<double>(0, headingStdDev);
	tiltDist = std::normal_distribution<double>(0, rollPitchStdDev);
	rotationDistribution = std::normal_distribution<double>(0, angularVelocityStdDev);

	imu = nh.advertise<sensor_msgs::Imu>("data", 1000);
}

void IMUModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	tf2::Quaternion rotation(vehicleState.getRotationNED());
	tf2::Vector3 angularVelocity = vehicleState.getAngularVelocityNED();
	//Add gaussian noise to the angular velocity
	for(unsigned int i = 0; i < 3; i++)
	{
		//Only apply the error if the std dev is greater than 0
		if(angularVelocityStdDev > 0)
		{
			double rotError = rotationDistribution(generator);
			angularVelocity[i] += rotError + angularVelocityBiasError[i];
		}
	}
	
	//Sample gaussian if the variance is greater than 0, otherwise set the random error to 0
	double rollRandomError = rollPitchStdDev > 0 ? tiltDist(generator) : 0;
	double pitchRandomError = rollPitchStdDev > 0 ? tiltDist(generator) : 0;
	double headingRandomError = headingStdDev > 0 ? headingDist(generator) : 0;

	//Get extrinsic RPY (same as intrinsic YPR)
	tf2::Matrix3x3 rotMatrix(rotation);
	double yaw, pitch,roll;
	rotMatrix.getRPY(roll, pitch, yaw);

	//Add error to the rpy and set extrinsic RPY (same as intrinsic YPR).
	//The idea behind this is to add heading error independently from roll and pitch noise.
	//heading is mainly determined based on magnetic fields and a compass and roll and pitch with accelerameters.
	//So the noise profile is different. This implementation prevents roll and pitch noise from affecting heading.
	//In a real IMU there is sensor fusion so roll and pitch would probably affect the heading, but without having a good
	//idea of that relationship it was avoided altogether.

	//There is probably a better way to implement this.
	rotation.setRPY(roll + rollRandomError + rollBiasError,
					pitch + pitchRandomError + pitchBiasError,
					yaw + headingRandomError + headingBiasError);

	sensor_msgs::ImuPtr data(new sensor_msgs::Imu);

	data->header.stamp = lastTime;
  	data->header.frame_id = "world_ned";

	if(headingActive) {
		data->orientation.x = rotation.x();
		data->orientation.y = rotation.y();
		data->orientation.z = rotation.z();
		data->orientation.w = rotation.w();
	
		data->orientation_covariance[0] = rollPitchStdDev * rollPitchStdDev;
		data->orientation_covariance[1] = 0;
		data->orientation_covariance[2] = 0;

		data->orientation_covariance[3] = 0;
		data->orientation_covariance[4] = rollPitchStdDev * rollPitchStdDev;
		data->orientation_covariance[5] = 0;

		data->orientation_covariance[6] = 0;
		data->orientation_covariance[7] = 0;
		data->orientation_covariance[8] = headingStdDev * headingStdDev;
	} else {
		data->orientation.x = std::numeric_limits<double>::quiet_NaN();
		data->orientation.y = std::numeric_limits<double>::quiet_NaN();
		data->orientation.z = std::numeric_limits<double>::quiet_NaN();
		data->orientation.w = std::numeric_limits<double>::quiet_NaN();
	
		data->orientation_covariance[0] = -1;
		data->orientation_covariance[1] = -1;
		data->orientation_covariance[2] = -1;

		data->orientation_covariance[3] = -1;
		data->orientation_covariance[4] = -1;
		data->orientation_covariance[5] = -1;

		data->orientation_covariance[6] = -1;
		data->orientation_covariance[7] = -1;
		data->orientation_covariance[8] = -1;
	}

	if(angVelActive) {
		data->angular_velocity.x = angularVelocity.x();
		data->angular_velocity.y = angularVelocity.y();
		data->angular_velocity.z = angularVelocity.z();

		data->angular_velocity_covariance[0] = angularVelocityStdDev * angularVelocityStdDev;
		data->angular_velocity_covariance[1] = 0;
		data->angular_velocity_covariance[2] = 0;

		data->angular_velocity_covariance[3] = 0;
		data->angular_velocity_covariance[4] = angularVelocityStdDev * angularVelocityStdDev;
		data->angular_velocity_covariance[5] = 0;

		data->angular_velocity_covariance[6] = 0;
		data->angular_velocity_covariance[7] = 0;
		data->angular_velocity_covariance[8] = angularVelocityStdDev * angularVelocityStdDev;
	} else {
		data->angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
		data->angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
		data->angular_velocity.z = std::numeric_limits<double>::quiet_NaN();

		data->angular_velocity_covariance[0] = -1;
		data->angular_velocity_covariance[1] = -1;
		data->angular_velocity_covariance[2] = -1;

		data->angular_velocity_covariance[3] = -1;
		data->angular_velocity_covariance[4] = -1;
		data->angular_velocity_covariance[5] = -1;

		data->angular_velocity_covariance[6] = -1;
		data->angular_velocity_covariance[7] = -1;
		data->angular_velocity_covariance[8] = -1;
	}


	data->linear_acceleration_covariance[0] = -1;

	imu.publish(data);
}