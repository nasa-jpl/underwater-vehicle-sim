#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"

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

    std::vector<tf2::Vector3> positions;
    positions.push_back(tf2::Vector3(0,0,0));
    positions.push_back(tf2::Vector3(10,10,10));
    positions.push_back(tf2::Vector3(-20,10,10));
    positions.push_back(tf2::Vector3(-20,30,10));
    positions.push_back(tf2::Vector3(-20,30,5));
    positions.push_back(tf2::Vector3(-20,30,95));

    for(tf2::Vector3 pos : positions)
    {
        state.setPosition(pos);
        module.update("v1", time, state);
        ros::spinOnce();
    }
   
    double maxTemp = 20;
    double maxSalt = 25;
    double maxDye = 30;
    double zeroDistance = 200;

    ASSERT_EQ(positions.size(), receivedMessages.size());

    for(unsigned int i = 0; i < receivedMessages.size(); i++)
    {
        double distance = positions[i].distance(tf2::Vector3(0,0,0));
        EXPECT_NEAR(maxTemp * (zeroDistance - distance) / zeroDistance, receivedMessages[i].temp, 0.0001);
        EXPECT_NEAR(maxSalt * (zeroDistance - distance) / zeroDistance, receivedMessages[i].salt, 0.0001);
        EXPECT_NEAR(maxDye * (zeroDistance - distance) / zeroDistance, receivedMessages[i].dye, 0.0001);
    }
}


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "data_broadcasting_module_test");

    return RUN_ALL_TESTS();
}
