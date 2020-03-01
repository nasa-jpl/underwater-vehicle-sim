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
    nhPriv.param("water_velocity_percent_error", waterVelocityPercentError, 0.01); //default is 1% of measured value
    nhPriv.param("water_velocity_random_error", waterVelocityRandomError, 0.003); //default is 0.3cm/s
    nhPriv.param("water_velocity_bias_error", waterVelocityBiasError, 0.0); //default is 0 m/s
    
    nhPriv.param("bottom_velocity_percent_error", bottomVelocityPercentError, 0.001); //default is 1% of measured value
    nhPriv.param("bottom_velocity_random_error", waterVelocityRandomError, 0.003); //default is 0.3cm/s
    nhPriv.param("bottom_velocity_bias_error", bottomVelocityBiasError, 0.0); //default is 0 m/s

    nhPriv.param("bottom_range_random_error", bottomRangeRandomError, 0.1); //default is 0.1 meters
    nhPriv.param("bottom_range_bias_error", bottomRangeBiasError, 0.0); //default is 0 meters

    nhPriv.param("bottom_lock_range", bottomLockRange, 10.0); //default is 10 meters

    int randomSeed;
    if(nhPriv.getParam("random_seed", randomSeed))
    {
        generator.seed(randomSeed);
    }
    bottomRangeDistribution = std::normal_distribution<double>(bottomRangeBiasError, bottomRangeRandomError);

    dvl = nh.advertise<underwater_vehicle_msgs::DVL>("data", 1000);
}

void DVLModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
    underwater_vehicle_msgs::DVLPtr dvlMsg(new underwater_vehicle_msgs::DVL);
    dvlMsg->header.frame_id = "world_ned";
    dvlMsg->header.stamp = lastTime;
    dvlMsg->name = name;

    tf2::Vector3 linearVelocity = vehicleState.getLinearVelocityNED();

    //Calculate bottom range
    double rangeReading = modelData.depth - vehicleState.getPositionNED().getZ();

    if(rangeReading > bottomLockRange)
    {
        dvlMsg->velocity_reference = dvlMsg->VELOCITY_REFERENCE_WATER;
        dvlMsg->range = -1;

        dvlMsg->velocity.x = linearVelocity[0];
        dvlMsg->velocity.y = linearVelocity[1];
        dvlMsg->velocity.z = linearVelocity[2];
        dvlMsg->velocity_covariance[0] = std::pow(dvlMsg->velocity.x * waterVelocityPercentError + waterVelocityRandomError, 2);
        dvlMsg->velocity_covariance[4] = std::pow(dvlMsg->velocity.y * waterVelocityPercentError + waterVelocityRandomError, 2);
        dvlMsg->velocity_covariance[8] = std::pow(dvlMsg->velocity.z * waterVelocityPercentError + waterVelocityRandomError, 2);

        if(waterVelocityPercentError > 0 || waterVelocityRandomError > 0)
        {
            std::normal_distribution<double> waterVelocityDistributionX(waterVelocityBiasError, dvlMsg->velocity.x * waterVelocityPercentError + waterVelocityRandomError);
            std::normal_distribution<double> waterVelocityDistributionY(waterVelocityBiasError, dvlMsg->velocity.y * waterVelocityPercentError + waterVelocityRandomError);
            std::normal_distribution<double> waterVelocityDistributionZ(waterVelocityBiasError, dvlMsg->velocity.z * waterVelocityPercentError + waterVelocityRandomError);

            double error = waterVelocityDistributionX(generator);
            dvlMsg->velocity.x += error;
            dvlMsg->velocity.y += waterVelocityDistributionY(generator);
            dvlMsg->velocity.z += waterVelocityDistributionZ(generator);
        }
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

        dvlMsg->velocity.x = linearVelocity[0] + modelData.u;
        dvlMsg->velocity.y = linearVelocity[1] + modelData.v;
        dvlMsg->velocity.z = linearVelocity[2] + modelData.w;
        dvlMsg->velocity_covariance[0] = std::pow(dvlMsg->velocity.x * bottomVelocityPercentError + bottomVelocityRandomError, 2);
        dvlMsg->velocity_covariance[4] = std::pow(dvlMsg->velocity.y * bottomVelocityPercentError + bottomVelocityRandomError, 2);
        dvlMsg->velocity_covariance[8] = std::pow(dvlMsg->velocity.z * bottomVelocityPercentError + bottomVelocityRandomError, 2);

        if(bottomVelocityPercentError > 0 || bottomVelocityRandomError > 0)
        {
            std::normal_distribution<double> bottomVelocityDistributionX(bottomVelocityBiasError, dvlMsg->velocity.x * bottomVelocityPercentError + bottomVelocityRandomError);
            std::normal_distribution<double> bottomVelocityDistributionY(bottomVelocityBiasError, dvlMsg->velocity.y * bottomVelocityPercentError + bottomVelocityRandomError);
            std::normal_distribution<double> bottomVelocityDistributionZ(bottomVelocityBiasError, dvlMsg->velocity.z * bottomVelocityPercentError + bottomVelocityRandomError);

            dvlMsg->velocity.x += bottomVelocityDistributionX(generator);
            dvlMsg->velocity.y += bottomVelocityDistributionY(generator);
            dvlMsg->velocity.z += bottomVelocityDistributionZ(generator);
        }
    }

    dvl.publish(dvlMsg);
}