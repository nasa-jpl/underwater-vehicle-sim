#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "data_server/GetData.h"
#include "model_server/GetModelData.h"

ros::ServiceClient client;
ros::ServiceClient modelClient;

TEST(DataRecorderModule, TestDataRecording){
        //Initalize ROS node handle
        ros::NodeHandle nh;
        client = nh.serviceClient<data_server::GetData>("data_server/get");
        modelClient = nh.serviceClient<model_server::GetModelData>("get_model_data");


        bool exists = modelClient.waitForExistence(ros::Duration(5));
        ASSERT_TRUE(exists); //check for the model server service existance

        ros::Duration(5).sleep();

        data_server::GetData retrievedData;
        retrievedData.request.name = "v1";
        retrievedData.request.start_time = ros::Time(1728000);
        retrievedData.request.end_time = ros::Time(1728010);

        client.call(retrievedData);


        ASSERT_TRUE(retrievedData.response.time.size() > 0);
        float prevTime = retrievedData.response.time[1].toSec();

        for(unsigned int i = 2; i < retrievedData.response.time.size(); i++)
        {
            ASSERT_NEAR(1.0, retrievedData.response.time[i].toSec() - prevTime, 0.1);

            ASSERT_FLOAT_EQ(0, retrievedData.response.temp[i]);
            ASSERT_FLOAT_EQ(4.567, retrievedData.response.salt[i]);
            ASSERT_FLOAT_EQ(5.678, retrievedData.response.dye[i]);
            prevTime = retrievedData.response.time[i].toSec();
        }
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "data_recorder_module_test");

  return RUN_ALL_TESTS();
}
