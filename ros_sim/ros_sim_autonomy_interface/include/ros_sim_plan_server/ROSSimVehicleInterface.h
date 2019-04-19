#ifndef ROS_SIM_VEHICLE_INTERFACE_H
#define ROS_SIM_VEHICLE_INTERFACE_H

#include "ros/ros.h"

#include "underwater_planner/GoalStatus.h"
#include "underwater_planner/VehicleInterface.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "tf2_ros/transform_listener.h"

#include "nav_msgs/Odometry.h"

class ROSSimVehicleInterface : public underwater_autonomy::VehicleInterface
{
public:
    ROSSimVehicleInterface(VehicleInfo info);
    ~ROSSimVehicleInterface() override = default;

    void sendGoalStatus(underwater_autonomy::GoalStatus status) override;
    void log(underwater_autonomy::LogLevel level, std::string string) override;
    void registerDataCallback(std::function<void(const underwater_autonomy::PlannerData&)> cb) override;

    underwater_autonomy::VehiclePose getPosition() const override;

private:
    void receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg);
    void navigationFilterCallback(const nav_msgs::Odometry odo);
    
private:
    VehicleInfo info;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    std::vector<std::function<void(const underwater_autonomy::PlannerData&)>> dataCallbacks;

    ros::Publisher goalPub;
    ros::Subscriber dataSub;
    ros::Subscriber poseSub;

    underwater_autonomy::VehiclePose currentPose;
};

#endif