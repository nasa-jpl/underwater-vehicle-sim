#ifndef SAMPLE_MODULE_H
#define SAMPLE_MODULE_H

#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "vehicles/GeneralModule.h"
#include "underwater_vehicle_msgs/RequestSample.h"

#include "ocean_models/model_interface/ModelData.h"

class SampleModule : public GeneralModule
{

public:
    SampleModule(std::string name);
    ~SampleModule() {}

    void update(const ros::Time& lastTime, VehicleState& vehicleState, ocean_models::ModelData& modelData);

private:
bool requestSample(underwater_vehicle_msgs::RequestSample::Request  &req,
                   underwater_vehicle_msgs::RequestSample::Response &res);
private:
    ros::Publisher takeSample;
	ros::ServiceServer requestSampleServer;

    ros::Time lastTimeUpdate;
    tf2::Vector3 lastPosition;
    ocean_models::ModelData lastModelData;
};

#endif
