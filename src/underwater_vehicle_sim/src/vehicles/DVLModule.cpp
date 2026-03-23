#include "vehicles/DVLModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "underwater_vehicle_msgs/DVL.h"

using namespace ocean_model_interfaces;

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
    underwater_vehicle_msgs::DVLPtr dvlMsgWater(new underwater_vehicle_msgs::DVL);
    underwater_vehicle_msgs::DVLPtr dvlMsgGround(new underwater_vehicle_msgs::DVL);
    dvlMsgWater->header.frame_id = "world_ned";
    dvlMsgWater->header.stamp = lastTime;
    dvlMsgWater->name = name;

    dvlMsgGround->header.frame_id = "world_ned";
    dvlMsgGround->header.stamp = lastTime;
    dvlMsgGround->name = name;

    //Calculate bottom range
    double rangeReading = modelData.depth - vehicleState.getPositionNED().getZ();

    dvlMsgWater->velocity_reference = dvlMsgWater->VELOCITY_REFERENCE_WATER;
    dvlMsgWater->range = -1;

    tf2::Vector3 linearVelocity = vehicleState.getLinearVelocityNED(false);
    dvlMsgWater->velocity.x = linearVelocity[0];
    dvlMsgWater->velocity.y = linearVelocity[1];
    dvlMsgWater->velocity.z = linearVelocity[2];
    dvlMsgWater->velocity_covariance[0] = std::pow(dvlMsgWater->velocity.x * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);
    dvlMsgWater->velocity_covariance[4] = std::pow(dvlMsgWater->velocity.y * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);
    dvlMsgWater->velocity_covariance[8] = std::pow(dvlMsgWater->velocity.z * waterVelocityScaleError, 2) + std::pow(waterVelocityRandomError, 2);

    if(waterVelocityScaleError > 0)
    {
        std::normal_distribution<double> waterVelocityScaleDistributionX(0, dvlMsgWater->velocity.x * waterVelocityScaleError);
        std::normal_distribution<double> waterVelocityScaleDistributionY(0, dvlMsgWater->velocity.y * waterVelocityScaleError);
        std::normal_distribution<double> waterVelocityScaleDistributionZ(0, dvlMsgWater->velocity.z * waterVelocityScaleError);
        dvlMsgWater->velocity.x += waterVelocityScaleDistributionX(generator);
        dvlMsgWater->velocity.y += waterVelocityScaleDistributionY(generator);
        dvlMsgWater->velocity.z += waterVelocityScaleDistributionZ(generator);
    }

    if(waterVelocityRandomError > 0) {
        dvlMsgWater->velocity.x += waterVelocityDistribution(generator);
        dvlMsgWater->velocity.y += waterVelocityDistribution(generator);
        dvlMsgWater->velocity.z += waterVelocityDistribution(generator);
    }

    dvlMsgWater->velocity.x += waterVelocityBiasError[0];
    dvlMsgWater->velocity.y += waterVelocityBiasError[1];
    dvlMsgWater->velocity.z += waterVelocityBiasError[2];
    
    if(rangeReading <= bottomLockRange)
    {
        dvlMsgGround->velocity_reference = dvlMsgGround->VELOCITY_REFERENCE_BOTTOM;
        //Only apply error if the std dev of the distribution is positive
        if(bottomRangeRandomError > 0)
        {
            rangeReading += bottomRangeDistribution(generator);
        }
        dvlMsgGround->range = rangeReading;

        tf2::Vector3 linearVelocity = vehicleState.getLinearVelocityNED(true);
        dvlMsgGround->velocity.x = linearVelocity[0];
        dvlMsgGround->velocity.y = linearVelocity[1];
        dvlMsgGround->velocity.z = linearVelocity[2];
        dvlMsgGround->velocity_covariance[0] = std::pow(dvlMsgGround->velocity.x * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);
        dvlMsgGround->velocity_covariance[4] = std::pow(dvlMsgGround->velocity.y * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);
        dvlMsgGround->velocity_covariance[8] = std::pow(dvlMsgGround->velocity.z * bottomVelocityScaleError, 2) + std::pow(bottomVelocityRandomError, 2);

        if(bottomVelocityScaleError > 0)
        {
            std::normal_distribution<double> bottomVelocityScaleDistributionX(0, dvlMsgGround->velocity.x * bottomVelocityScaleError);
            std::normal_distribution<double> bottomVelocityScaleDistributionY(0, dvlMsgGround->velocity.y * bottomVelocityScaleError);
            std::normal_distribution<double> bottomVelocityScaleDistributionZ(0, dvlMsgGround->velocity.z * bottomVelocityScaleError);
            dvlMsgGround->velocity.x += bottomVelocityScaleDistributionX(generator);
            dvlMsgGround->velocity.y += bottomVelocityScaleDistributionY(generator);
            dvlMsgGround->velocity.z += bottomVelocityScaleDistributionZ(generator);
        }

        if(bottomVelocityRandomError > 0) {
            dvlMsgGround->velocity.x += bottomVelocityDistribution(generator);
            dvlMsgGround->velocity.y += bottomVelocityDistribution(generator);
            dvlMsgGround->velocity.z += bottomVelocityDistribution(generator);
        }

        dvlMsgGround->velocity.x += bottomVelocityBiasError[0];
        dvlMsgGround->velocity.y += bottomVelocityBiasError[1];
        dvlMsgGround->velocity.z += bottomVelocityBiasError[2];
        dvl.publish(dvlMsgGround);
    }
    dvl.publish(dvlMsgWater);
}