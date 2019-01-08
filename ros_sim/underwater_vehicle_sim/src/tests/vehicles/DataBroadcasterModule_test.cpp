#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "data_server/GetData.h"
#include "model_server/GetModelData.h"

#include "underwater_vehicle_msgs/VehicleData.h"
#include "vehicles/DataBroadcasterModule.h"

std::vector<underwater_vehicle_msgs::VehicleData> receivedMessages;

void dataCallback(const underwater_vehicle_msgs::VehicleDataPtr& vel)
{
    receivedMessages.push_back(*vel);
}

TEST(DataBroadcasterModule, TestDataRecording){
    //Initalize ROS node handle
    ros::NodeHandle nh("/vehicles/v1");
    VehicleState state(nh);
    DataBroadcasterModule module("data_broadcaster", nh, "v1");

    ros::Subscriber dataSub = nh.subscribe("data_broadcaster/data", 1, &dataCallback);

    ros::Time time(0);
    double centerX = 5;
    double centerY = 20;
    double centerZ = -10;

    double maxTemp = 20;
    double maxSalt = 25;
    double maxDye = 30;
    double zeroDistance = 200;

    std::vector<tf2::Vector3> enuPositions;
    enuPositions.push_back(tf2::Vector3(0,0,0));
    enuPositions.push_back(tf2::Vector3(5, 20,-10));
    enuPositions.push_back(tf2::Vector3(-20, 10, -10));
    enuPositions.push_back(tf2::Vector3(-20, 30, -10));
    enuPositions.push_back(tf2::Vector3(-20, 30, -5));
    enuPositions.push_back(tf2::Vector3(-20, 30, -95));

    //Send data to the module to be broadcast
    for(tf2::Vector3 pos : enuPositions)
    {
        double distance = pos.distance(tf2::Vector3(centerX, centerY, centerZ));
        
        ModelData data;
        data.temp = maxTemp * (zeroDistance - distance) / zeroDistance;
        data.salt = maxSalt * (zeroDistance - distance) / zeroDistance;
        data.dye = maxDye * (zeroDistance - distance) / zeroDistance;

        state.setPositionENU(pos);
        module.update("v1", time, state, data);
        ros::spinOnce();
    }

    ASSERT_EQ(enuPositions.size(), receivedMessages.size());

    for(unsigned int i = 0; i < receivedMessages.size(); i++)
    {
        //Data sent from the module should match the data sent to the module
        double distance = enuPositions[i].distance(tf2::Vector3(centerX, centerY, centerZ));
        EXPECT_NEAR(maxTemp * (zeroDistance - distance) / zeroDistance, receivedMessages[i].temp, 0.0001);
        EXPECT_NEAR(maxSalt * (zeroDistance - distance) / zeroDistance, receivedMessages[i].salt, 0.0001);
        EXPECT_NEAR(maxDye * (zeroDistance - distance) / zeroDistance, receivedMessages[i].dye, 0.0001);
    }
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
    ros::init(argc, argv, "data_broadcasting_module_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
