#include "vehicles/DVLModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "underwater_vehicle_msgs/DVL.h"

using namespace ocean_models;

DVLModule::DVLModule(std::string name) :
    GeneralModule(name, "DVL")
{
    ros::NodeHandle nhPriv("~/" + name);
    nhPriv.param("water_velocity_scale_error", waterVelocityScaleError, 0.01); //default is 1% of measured value
    nhPriv.param("water_velocity_random_error", waterVelocityRandomError, 0.003); //default is 0.3cm/s
    nhPriv.param("water_velocity_bias_error", waterVelocityBiasError, {0.0, 0.0, 0.0}); //default is 0 m/s
    
    nhPriv.param("bottom_velocity_scale_error", bottomVelocityScaleError, 0.001); //default is 1% of measured value
    nhPriv.param("bottom_velocity_random_error", bottomVelocityRandomError, 0.003); //default is 0.3cm/s
    nhPriv.param("bottom_velocity_bias_error", bottomVelocityBiasError, {0.0, 0.0, 0.0}); //default is 0 m/s

    nhPriv.param("bottom_range_random_error", bottomRangeRandomError, 0.1); //default is 0.1 meters
    nhPriv.param("bottom_range_bias_error", bottomRangeBiasError, 0.0); //default is 0 meters

    nhPriv.param("bottom_lock_range", bottomLockRange, 10.0); //default is 10 meters

    int randomSeed;
    if(nhPriv.getParam("random_seed", randomSeed))
    {
        generator.seed(randomSeed);
    }
    bottomRangeDistribution = std::normal_distribution<double>(bottomRangeBiasError, bottomRangeRandomError);
    waterVelocityDistribution = std::normal_distribution<double>(0, waterVelocityRandomError);
    bottomVelocityDistribution = std::normal_distribution<double>(0, bottomVelocityRandomError);

    dvl = nh.advertise<underwater_vehicle_msgs::DVL>("data", 1000);
}

void DVLModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
    underwater_vehicle_msgs::DVLPtr dvlMsg(new underwater_vehicle_msgs::DVL);
    dvlMsg->header.frame_id = "world_ned";
    dvlMsg->header.stamp = lastTime;
    dvlMsg->name = name;

    //Calculate bottom range
    double rangeReading = modelData.depth - vehicleState.getPositionNED().getZ();

    if(rangeReading > bottomLockRange)
    {
        dvlMsg->velocity_reference = dvlMsg->VELOCITY_REFERENCE_WATER;
        dvlMsg->range = -1;

        tf2::Vector3 linearVelocity = vehicleState.getLinearVelocityNED(false);
        dvlMsg->velocity.x = linearVelocity[0];
        dvlMsg->velocity.y = linearVelocity[1];
        dvlMsg->velocity.z = linearVelocity[2];
        dvlMsg->velocity_covariance[0] = std::pow(dvlMsg->velocity.x * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);
        dvlMsg->velocity_covariance[4] = std::pow(dvlMsg->velocity.y * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);
        dvlMsg->velocity_covariance[8] = std::pow(dvlMsg->velocity.z * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);

        if(waterVelocityScaleError > 0)
        {
            std::normal_distribution<double> waterVelocityScaleDistributionX(0, dvlMsg->velocity.x * waterVelocityScaleError);
            std::normal_distribution<double> waterVelocityScaleDistributionY(0, dvlMsg->velocity.y * waterVelocityScaleError);
            std::normal_distribution<double> waterVelocityScaleDistributionZ(0, dvlMsg->velocity.z * waterVelocityScaleError);
            dvlMsg->velocity.x += waterVelocityScaleDistributionX(generator);
            dvlMsg->velocity.y += waterVelocityScaleDistributionY(generator);
            dvlMsg->velocity.z += waterVelocityScaleDistributionZ(generator);
        }

        if(waterVelocityRandomError > 0) {
            dvlMsg->velocity.x += waterVelocityDistribution(generator);
            dvlMsg->velocity.y += waterVelocityDistribution(generator);
            dvlMsg->velocity.z += waterVelocityDistribution(generator);
        }

        dvlMsg->velocity.x += waterVelocityBiasError[0];
        dvlMsg->velocity.y += waterVelocityBiasError[1];
        dvlMsg->velocity.z += waterVelocityBiasError[2];
    }
    else
    {
        dvlMsg->velocity_reference = dvlMsg->VELOCITY_REFERENCE_BOTTOM;
        //Only apply error if the std dev of the distribution is positive
        if(bottomRangeRandomError > 0)
        {
            rangeReading += bottomRangeDistribution(generator);
        }
        dvlMsg->range = rangeReading;

        tf2::Vector3 linearVelocity = vehicleState.getLinearVelocityNED(true);
        dvlMsg->velocity.x = linearVelocity[0];
        dvlMsg->velocity.y = linearVelocity[1];
        dvlMsg->velocity.z = linearVelocity[2];
        dvlMsg->velocity_covariance[0] = std::pow(dvlMsg->velocity.x * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);
        dvlMsg->velocity_covariance[4] = std::pow(dvlMsg->velocity.y * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);
        dvlMsg->velocity_covariance[8] = std::pow(dvlMsg->velocity.z * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);

        if(bottomVelocityScaleError > 0)
        {
            std::normal_distribution<double> bottomVelocityScaleDistributionX(0, dvlMsg->velocity.x * bottomVelocityScaleError);
            std::normal_distribution<double> bottomVelocityScaleDistributionY(0, dvlMsg->velocity.y * bottomVelocityScaleError);
            std::normal_distribution<double> bottomVelocityScaleDistributionZ(0, dvlMsg->velocity.z * bottomVelocityScaleError);
            dvlMsg->velocity.x += bottomVelocityScaleDistributionX(generator);
            dvlMsg->velocity.y += bottomVelocityScaleDistributionY(generator);
            dvlMsg->velocity.z += bottomVelocityScaleDistributionZ(generator);
        }

        if(bottomVelocityRandomError > 0) {
            dvlMsg->velocity.x += bottomVelocityDistribution(generator);
            dvlMsg->velocity.y += bottomVelocityDistribution(generator);
            dvlMsg->velocity.z += bottomVelocityDistribution(generator);
        }

        dvlMsg->velocity.x += bottomVelocityBiasError[0];
        dvlMsg->velocity.y += bottomVelocityBiasError[1];
        dvlMsg->velocity.z += bottomVelocityBiasError[2];
    }

    dvl.publish(dvlMsg);
}