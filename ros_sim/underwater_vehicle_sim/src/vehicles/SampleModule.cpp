#include "vehicles/SampleModule.h"

#include <limits>

#include "ros/ros.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "underwater_vehicle_msgs/TakeSample.h"

using namespace ocean_models;

SampleModule::SampleModule(std::string name) :
    GeneralModule(name, "Sample")
{
    takeSample = nh.advertise<underwater_vehicle_msgs::TakeSample>("take_sample", 1000);
    requestSampleServer = nh.advertiseService("request_sample", &SampleModule::requestSample, this);
}

void SampleModule::update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData) 
{
    lastTimeUpdate = lastTime;
    lastPosition = vehicleState.getPositionNED();
    lastModelData = modelData;
}

bool SampleModule::requestSample(underwater_vehicle_msgs::RequestSample::Request  &req,
                                 underwater_vehicle_msgs::RequestSample::Response &res)
{
    underwater_vehicle_msgs::TakeSamplePtr sampleMsg(new underwater_vehicle_msgs::TakeSample);

    sampleMsg->header.stamp = lastTimeUpdate;
    sampleMsg->data = lastModelData.dye;
    sampleMsg->location.x = lastPosition.x();
    sampleMsg->location.y = lastPosition.y();
    sampleMsg->location.z = lastPosition.z();

    takeSample.publish(sampleMsg);

    ROS_INFO("Sample Requested at location: %f %f %f with value %f at time %f", 
              sampleMsg->location.x, 
              sampleMsg->location.y, 
              sampleMsg->location.z, 
              sampleMsg->data, 
              sampleMsg->header.stamp.toSec());
    return true;
}