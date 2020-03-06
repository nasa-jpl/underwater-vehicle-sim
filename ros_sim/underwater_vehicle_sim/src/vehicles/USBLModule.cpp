#include "vehicles/USBLModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "underwater_vehicle_msgs/USBL.h"

using namespace ocean_models;

USBLModule::USBLModule(std::string name) :
	GeneralModule(name, "USBL")
{
	ros::NodeHandle nhPriv("~/" + name);
	nhPriv.param("bearing_random_error", bearingRandomError, 0.0174533); //default is 1 deg in radians
	nhPriv.param("bearing_bias_error", bearingBiasError, 0.0); //default is 0 deg in rad

	nhPriv.param("range_random_error", rangeRandomError, 0.005); //default is 0.5% of range
	nhPriv.param("range_bias_error", rangeBiasError, 0.0); //default is 0 m

	nhPriv.param("valid_bearing_distance", validBearingDistance, 1000.0); //default is 1000 m
	nhPriv.param("valid_range_distance", validRangeDistance, 5000.0); //default is 5000 m

	nhPriv.param("bad_range_probability", badRangeProbability, 0.0);
	nhPriv.param("bad_range_error", badRangeError, 0.0);

	nhPriv.param("beacon_x", beaconX, 0.0);
	nhPriv.param("beacon_y", beaconY, 0.0);
	nhPriv.param("beacon_z", beaconZ, 0.0);
	
	int randomSeed;
	if(nhPriv.getParam("random_seed", randomSeed))
	{
		generator.seed(randomSeed);
	}
	bearingDistribution = std::normal_distribution<double>(0, bearingRandomError);
	badRangeRandom = std::uniform_real_distribution<double>(0, 1.0);
	badRangeDistribution = std::uniform_real_distribution<double>(-badRangeError, badRangeError);


	usbl = nh.advertise<underwater_vehicle_msgs::USBL>("data", 1000);
}

void USBLModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
	double xDiff = vehicleState.getPositionNED().getX() - beaconX;
	double yDiff = vehicleState.getPositionNED().getY() - beaconY;
	double zDiff = vehicleState.getPositionNED().getZ() - beaconZ;
	double trueRange = std::sqrt(std::pow(xDiff, 2) +
							 	 std::pow(yDiff, 2) +
							 	 std::pow(zDiff, 2));

	double range = std::numeric_limits<double>::quiet_NaN();
	double bearing = std::numeric_limits<double>::quiet_NaN();
	if(trueRange <= validRangeDistance) 
	{
		if(badRangeRandom(generator) <= badRangeProbability)
		{
			range = trueRange;
			range += badRangeDistribution(generator);
		}
		else
		{
			range = trueRange;

			if(rangeRandomError > 0)
			{
				std::normal_distribution<double> rangeDistribution(0, trueRange * rangeRandomError);
				double rangeError = rangeDistribution(generator);
				range += rangeError;
			}

			range += trueRange * rangeBiasError;
		}
	}

	if(trueRange <= validBearingDistance)
	{
		bearing = atan2(yDiff, xDiff);
		bearing += bearingBiasError;

		if(bearingRandomError > 0)
		{
			double error = bearingDistribution(generator);
			bearing += error;
		}
	}

	underwater_vehicle_msgs::USBLPtr usblMsg(new underwater_vehicle_msgs::USBL);

	usblMsg->header.frame_id = "world_ned";
	usblMsg->header.stamp = lastTime;

	usblMsg->name = name;
	usblMsg->beacon_x = beaconX;
	usblMsg->beacon_y = beaconY;
	usblMsg->beacon_z = beaconZ;
	usblMsg->range = range;
	usblMsg->bearing = bearing;

	usbl.publish(usblMsg);
}