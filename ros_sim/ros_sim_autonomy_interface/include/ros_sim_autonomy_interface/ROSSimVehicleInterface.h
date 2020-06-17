#ifndef ROS_SIM_VEHICLE_INTERFACE_H
#define ROS_SIM_VEHICLE_INTERFACE_H

#include "ros/ros.h"

#include "underwater_autonomy/planner/VehicleInterface.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "sensor_msgs/Imu.h"
#include "underwater_vehicle_msgs/USBL.h"
#include "underwater_vehicle_msgs/FloatMeasurement.h"
#include "underwater_vehicle_msgs/DVL.h"

#include "tf2_ros/transform_listener.h"

#include "nav_msgs/Odometry.h"

class ROSSimVehicleInterface : public underwater_autonomy::VehicleInterface
{
public:
    ROSSimVehicleInterface(VehicleInfo info);
    ~ROSSimVehicleInterface() override = default;

    void sendPlannerStatus(underwater_autonomy::PlannerStatus status) override;
    void log(underwater_autonomy::LogLevel level, std::string string) override;

    underwater_autonomy::VehiclePose getPosition() const override;
    double getTime() const override;

    VehicleInfo getVehicleInfo();

private:
    void initializeCallbacks();

    void receiveModelData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg);
    void receivePose(const ros::TimerEvent& event);
    void receiveIMU(sensor_msgs::Imu msgData);
    void receiveDepth(underwater_vehicle_msgs::FloatMeasurement msgData);
    void receiveUSBL(underwater_vehicle_msgs::USBL msgData);
    void receiveDVL(underwater_vehicle_msgs::DVL msgData);

    void receiveForwardThruster(underwater_vehicle_msgs::FloatMeasurement forwardData);
    void receiveLateralThruster(underwater_vehicle_msgs::FloatMeasurement lateralData);

    void navigationFilterCallback(const nav_msgs::Odometry odo);
    

private:
    VehicleInfo info;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    ros::Publisher goalPub;
    ros::Subscriber dataSub;
    ros::Subscriber poseSub;
    ros::Subscriber imuSub;
    ros::Subscriber usblSub;
    ros::Subscriber depthSub;
    ros::Subscriber dvlSub;
    ros::Subscriber forwardThrusterSub;
    ros::Subscriber lateralThrusterSub;

    underwater_autonomy::VehiclePose currentPose;

    ros::Timer tfTimer;
    double lastTFTime;
    underwater_autonomy::VehiclePose lastTfPose;
};

#endif