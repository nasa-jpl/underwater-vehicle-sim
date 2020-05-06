#ifndef DVL_MODULE_H
#define DVL_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

#include <random>

class DVLModule : public GeneralModule
{

public:
    DVLModule(std::string name);
    ~DVLModule() {}

    void update(const ros::Time& lastTime, VehicleState& vehicleState, ocean_models::ModelData& modelData);

private:
    
private:
    ros::Publisher dvl;

    /**
    *Error in velocity measurment with respect to the water
    */
    double waterVelocityRandomError;
    double waterVelocityScaleError;
    std::vector<double> waterVelocityBiasError;

    /**
    *Error in velocity measurment with respect to the bottom
    */
    double bottomVelocityRandomError;
    double bottomVelocityScaleError;
    std::vector<double> bottomVelocityBiasError;

    /**
    *Error in bottom range measurment
    */
    double bottomRangeRandomError;
    double bottomRangeBiasError;

    double bottomLockRange;

    std::default_random_engine generator;
    std::normal_distribution<double> bottomRangeDistribution;
    std::normal_distribution<double> waterVelocityDistribution;
    std::normal_distribution<double> bottomVelocityDistribution;

};

#endif
