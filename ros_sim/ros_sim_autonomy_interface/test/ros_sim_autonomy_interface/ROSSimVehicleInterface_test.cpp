#include <gtest/gtest.h>

#include <iostream>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "sensor_msgs/Imu.h"
#include "underwater_vehicle_msgs/USBL.h"
#include "underwater_vehicle_msgs/FloatMeasurement.h"
#include "underwater_vehicle_msgs/DVL.h"
#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include "underwater_autonomy/sensor_data_types/CommonDataTypes.h"

using namespace underwater_autonomy;


TEST(ROSSimNavigationFilter, IMUCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"IMU"};
    infoSrv.response.moduleNames = {"imu"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotHeading = false;
    DoubleSensorData lastHeading;
    bool gotAngVel = false;
    Vector3dData lastAngularVelocity;

    std::function<void(const DoubleSensorData&)> headingCB = [&](const DoubleSensorData& data) { lastHeading = data; 
                                                                                                 gotHeading = true; };
    std::function<void(const Vector3dData&)> angVelCB = [&](const Vector3dData& data) { lastAngularVelocity = data; 
                                                                                                    gotAngVel = true; };
    interface.registerDataCallback<DoubleSensorData>("heading", headingCB);
    interface.registerDataCallback<Vector3dData>("angular_velocity", angVelCB);

    ros::NodeHandle nh;
    ros::Publisher headingPub = nh.advertise<sensor_msgs::Imu>("imu/data", 1, true);
    sensor_msgs::Imu imu;

    tf2::Quaternion orientation;
    orientation.setRPY(0,0, M_PI);

    imu.orientation.x = orientation.getX();
    imu.orientation.y = orientation.getY();
    imu.orientation.z = orientation.getZ();
    imu.orientation.w = orientation.getW();
    imu.angular_velocity.x = 1;
    imu.angular_velocity.y = 2;
    imu.angular_velocity.z = 3;
    imu.header.stamp = ros::Time(10);

    headingPub.publish(imu);

    while(!gotHeading || !gotAngVel) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(10, lastHeading.time);
    EXPECT_DOUBLE_EQ(M_PI, lastHeading.data);

    EXPECT_DOUBLE_EQ(10, lastAngularVelocity.time);
    EXPECT_DOUBLE_EQ(1, lastAngularVelocity.data[0]);
    EXPECT_DOUBLE_EQ(2, lastAngularVelocity.data[1]);
    EXPECT_DOUBLE_EQ(3, lastAngularVelocity.data[2]);
}

TEST(ROSSimNavigationFilter, USBLCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"USBL"};
    infoSrv.response.moduleNames = {"usbl"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotUSBL = false;
    USBLSensorData lastUSBL;

    std::function<void(const USBLSensorData&)> usblCB = [&](const USBLSensorData& data) { lastUSBL = data; 
                                                                                          gotUSBL = true; };

    interface.registerDataCallback<USBLSensorData>("usbl", usblCB);

    ros::NodeHandle nh;
    ros::Publisher usblPub = nh.advertise<underwater_vehicle_msgs::USBL>("usbl/data", 1, true);
    underwater_vehicle_msgs::USBL usbl;

    usbl.range = 11.0;
    usbl.bearing = M_PI;
    usbl.header.stamp = ros::Time(10);

    usblPub.publish(usbl);

    while(!gotUSBL) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(10, lastUSBL.time);
    EXPECT_DOUBLE_EQ(11.0, lastUSBL.range);
    EXPECT_DOUBLE_EQ(M_PI, lastUSBL.bearing);
}

TEST(ROSSimNavigationFilter, DepthCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"Depth"};
    infoSrv.response.moduleNames = {"depth"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotDepth = false;
    DoubleSensorData lastDepth;

    std::function<void(const DoubleSensorData&)> depthCB = [&](const DoubleSensorData& data) { lastDepth = data; 
                                                                                               gotDepth = true; };

    interface.registerDataCallback<DoubleSensorData>("depth", depthCB);

    ros::NodeHandle nh;
    ros::Publisher depthPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("depth/data", 1, true);
    underwater_vehicle_msgs::FloatMeasurement depth;

    depth.data = 21.0;
    depth.header.stamp = ros::Time(10);

    depthPub.publish(depth);

    while(!gotDepth) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(10, lastDepth.time);
    EXPECT_DOUBLE_EQ(21.0, lastDepth.data);
}

TEST(ROSSimNavigationFilter, DVLCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"DVL"};
    infoSrv.response.moduleNames = {"dvl"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotDVL = false;
    DVLSensorData lastDVL;

    std::function<void(const DVLSensorData&)> dvlCB = [&](const DVLSensorData& data) { lastDVL = data; 
                                                                                       gotDVL = true; };

    interface.registerDataCallback<DVLSensorData>("dvl", dvlCB);

    ros::NodeHandle nh;
    ros::Publisher dvlPub = nh.advertise<underwater_vehicle_msgs::DVL>("dvl/data", 1, true);
    underwater_vehicle_msgs::DVL dvl;

    dvl.velocity.x = 10.0;
    dvl.velocity.y = 11.0;
    dvl.velocity.z = 12.0;
    dvl.velocity_reference = dvl.VELOCITY_REFERENCE_WATER;

    dvl.header.stamp = ros::Time(15);

    dvlPub.publish(dvl);

    while(!gotDVL) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(15, lastDVL.time);
    EXPECT_DOUBLE_EQ(10, lastDVL.x);
    EXPECT_DOUBLE_EQ(11, lastDVL.y);
    EXPECT_DOUBLE_EQ(12, lastDVL.z);
    EXPECT_EQ(DVLSensorData::VelocityReference::WATER, lastDVL.velocityReference);
}

TEST(ROSSimNavigationFilter, DataBroadcasterCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"DataBroadcaster"};
    infoSrv.response.moduleNames = {"data_broadcaster"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotSonarDepth = false;
    DoubleSensorData lastSonarDepth;

    bool gotTemp = false;
    DoubleSensorData lastTemp;

    bool gotSalt = false;
    DoubleSensorData lastSalt;

    bool gotDye = false;
    DoubleSensorData lastDye;

    bool gotPlume = false;
    DoubleSensorData lastPlume;

    std::function<void(const DoubleSensorData&)> sonarDepthCB = [&](const DoubleSensorData& data) { lastSonarDepth = data; 
                                                                                                    gotSonarDepth = true; };

    std::function<void(const DoubleSensorData&)> tempCB = [&](const DoubleSensorData& data) { lastTemp = data; 
                                                                                              gotTemp = true; };

    std::function<void(const DoubleSensorData&)> saltCB = [&](const DoubleSensorData& data) { lastSalt = data; 
                                                                                              gotSalt = true; };

    std::function<void(const DoubleSensorData&)> dyeCB = [&](const DoubleSensorData& data) { lastDye = data; 
                                                                                             gotDye = true; };

    std::function<void(const DoubleSensorData&)> plumeCB = [&](const DoubleSensorData& data) { lastPlume = data; 
                                                                                               gotPlume = true; };

    interface.registerDataCallback<DoubleSensorData>("sonar_depth", sonarDepthCB);
    interface.registerDataCallback<DoubleSensorData>("temp", tempCB);
    interface.registerDataCallback<DoubleSensorData>("salt", saltCB);
    interface.registerDataCallback<DoubleSensorData>("dye", dyeCB);
    interface.registerDataCallback<DoubleSensorData>("plume", plumeCB);

    ros::NodeHandle nh;
    ros::Publisher vehicleDataPub = nh.advertise<underwater_vehicle_msgs::VehicleData>("data_broadcaster/data", 1, true);
    underwater_vehicle_msgs::VehicleData vehicleData;

    vehicleData.sonarDepth = 10.0;
    vehicleData.temp = 11.0;
    vehicleData.salt = 12.0;
    vehicleData.dye = 13.0;
    vehicleData.time = ros::Time(14);

    vehicleDataPub.publish(vehicleData);

    while(!gotSonarDepth ||
          !gotTemp ||
          !gotSalt ||
          !gotDye ||
          !gotPlume) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(10, lastSonarDepth.data);
    EXPECT_DOUBLE_EQ(11, lastTemp.data);
    EXPECT_DOUBLE_EQ(12, lastSalt.data);
    EXPECT_DOUBLE_EQ(13, lastDye.data);
    EXPECT_DOUBLE_EQ(13, lastPlume.data);

    EXPECT_DOUBLE_EQ(14, lastSonarDepth.time);
    EXPECT_DOUBLE_EQ(14, lastTemp.time);
    EXPECT_DOUBLE_EQ(14, lastSalt.time);
    EXPECT_DOUBLE_EQ(14, lastDye.time);
    EXPECT_DOUBLE_EQ(14, lastPlume.time);
}

TEST(ROSSimNavigationFilter, CommandedVelocityCallback)
{
    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";
    infoSrv.response.moduleTypes = {"DVL"};
    infoSrv.response.moduleNames = {"dvl"};
    VehicleInfo info(infoSrv);

    ROSSimVehicleInterface interface(info);

    bool gotForward = false;
    DoubleSensorData lastForward;
    bool gotVertical = false;
    DoubleSensorData lastVertical;

    std::function<void(const DoubleSensorData&)> forwardCB = [&](const DoubleSensorData& data) { lastForward = data; 
                                                                                                 gotForward = true; };

    std::function<void(const DoubleSensorData&)> verticalCB = [&](const DoubleSensorData& data) { lastVertical = data; 
                                                                                                 gotVertical = true; };

    interface.registerDataCallback<DoubleSensorData>("commanded_forward_velocity", forwardCB);
    interface.registerDataCallback<DoubleSensorData>("commanded_vertical_velocity", verticalCB);

    ros::NodeHandle nh;
    ros::Publisher forwardPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("commanded_forward_velocity", 1, true);
    ros::Publisher verticalPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("commanded_vertical_velocity", 1, true);

    underwater_vehicle_msgs::FloatMeasurement forwardData;
    underwater_vehicle_msgs::FloatMeasurement verticalData;

    forwardData.data = 0.56;
    verticalData.data = 0.67;
    forwardData.header.stamp = ros::Time(16);
    verticalData.header.stamp = ros::Time(17);

    forwardPub.publish(forwardData);
    verticalPub.publish(verticalData);

    while(!gotForward && !gotVertical) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    EXPECT_DOUBLE_EQ(16, lastForward.time);
    EXPECT_DOUBLE_EQ(17, lastVertical.time);
    EXPECT_DOUBLE_EQ(0.56, lastForward.data);
    EXPECT_DOUBLE_EQ(0.67, lastVertical.data);
}

//Had issues doing this in the roslaunch file for this test. Not sure why.
//Normally this can be included in the roslaunch file with the following
//<node pkg="tf2_ros" type="static_transform_publisher" name="ned_publisher" args="0 0 0 1.57 0 3.14 world world_ned"/>
void broadcastStaticTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "world_ned";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, M_PI / 2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_sim_vehicle_interface_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
