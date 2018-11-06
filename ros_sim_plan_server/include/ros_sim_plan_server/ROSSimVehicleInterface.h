#ifndef ROS_SIM_VEHICLE_INTERFACE_H
#define ROS_SIM_VEHICLE_INTERFACE_H

#include "tf/transform_listener.h"

#include "planner_framework/VehicleInterface.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

class ROSSimVehicleInterface : VehicleInterface
{
public:
    ROSSimVehicleInterface(ros::NodeHandle& nh, VehicleInfo info);
    ~ROSSimVehicleInterface() {}

    void getData() override;
    void registerDataCallback(std::function<void(const PlannerData&)> cb) override;
    VehiclePose getPosition() override;

private:

    void receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg);

private:
    ros::NodeHandle& nh;
    VehicleInfo info;

    tf::TransformListener listener;

    std::vector<std::function<void(const PlannerData&)>> dataCallbacks;

    ros::Subscriber dataSub;

};

#endif