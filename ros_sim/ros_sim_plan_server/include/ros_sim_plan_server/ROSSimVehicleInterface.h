#ifndef ROS_SIM_VEHICLE_INTERFACE_H
#define ROS_SIM_VEHICLE_INTERFACE_H

#include "ros/ros.h"

#include "underwater_planner/GoalStatus.h"
#include "underwater_planner/VehicleInterface.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "tf2_ros/transform_listener.h"

class ROSSimVehicleInterface : public VehicleInterface
{
public:
    ROSSimVehicleInterface(ros::NodeHandle& nh, VehicleInfo info);
    ~ROSSimVehicleInterface() override = default;

    void sendGoalStatus(GoalStatus status) override;
    void log(LogLevel level, std::string string) override;
    void getData() override;
    void registerDataCallback(std::function<void(const PlannerData&)> cb) override;
    VehiclePose getPosition() override;

private:
    void receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg);

private:
    ros::NodeHandle& nh;
    VehicleInfo info;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    std::vector<std::function<void(const PlannerData&)>> dataCallbacks;

    ros::Publisher goalPub;
    ros::Subscriber dataSub;

};

#endif