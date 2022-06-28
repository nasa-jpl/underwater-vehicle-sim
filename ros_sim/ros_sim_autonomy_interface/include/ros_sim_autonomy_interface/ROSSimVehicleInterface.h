#ifndef ROS_SIM_VEHICLE_INTERFACE_H
#define ROS_SIM_VEHICLE_INTERFACE_H

#include "ros/ros.h"

#include "underwater_autonomy/behaviors/VehicleInterface.h"
#include "underwater_autonomy/util/LogData.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "sensor_msgs/Imu.h"
#include "underwater_vehicle_msgs/USBL.h"
#include "underwater_vehicle_msgs/FloatMeasurement.h"
#include "underwater_vehicle_msgs/DVL.h"
#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "tf2_ros/transform_listener.h"

#include "nav_msgs/Odometry.h"

class ROSSimVehicleInterface : public underwater_autonomy::VehicleInterface
{
public:
    ROSSimVehicleInterface(VehicleInfo info);
    ~ROSSimVehicleInterface() override = default;

    void log(underwater_autonomy::LogLevel level, std::string string) override;
    void log(std::string channel, underwater_autonomy::LogData data) override;

    underwater_autonomy::VehiclePose getPosition() const override;
    double getTime() const override;

    VehicleInfo getVehicleInfo();

private:
    void initializeCallbacks();

    void receiveModelData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg);
    void receiveIMU(sensor_msgs::Imu msgData);
    void receiveDepth(underwater_vehicle_msgs::FloatMeasurement msgData);
    void receiveUSBL(underwater_vehicle_msgs::USBL msgData);
    void receiveDVL(underwater_vehicle_msgs::DVL msgData);

    void receiveCommandedFowardVelocity(underwater_vehicle_msgs::FloatMeasurement commandedForwardVelocity);

    void navigationFilterCallback(const nav_msgs::Odometry odo);
    

private:
    VehicleInfo info;
    ros::NodeHandle nh;
    ros::Publisher statusPub;
    ros::Subscriber dataSub;
    ros::Subscriber imuSub;
    ros::Subscriber usblSub;
    ros::Subscriber poseSub;
    ros::Subscriber depthSub;
    ros::Subscriber dvlSub;
    ros::Subscriber forwardVelSub;
    ros::Subscriber verticalVelSub;

    underwater_autonomy::VehiclePose currentPose;

    std::map<std::string, ros::Publisher> logPublishers;
};

#endif