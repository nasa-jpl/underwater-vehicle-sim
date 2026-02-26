#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"

ros::ServiceClient client;

TEST(ClockServerNode, TestTime){
	//Initalize ROS node handle
  	ros::NodeHandle n;

  	//wait for the first non-zero time to start
  	while(ros::Time::now().toSec() < 0.000001) {};

    ros::Duration dur(10);

    //loop for 10 seconds in ros time
  	ros::WallTime startWallTime = ros::WallTime::now();
    dur.sleep();
  	ros::WallTime endWallTime = ros::WallTime::now();
  	ros::Time endTime = ros::Time::now();

  	ros::WallDuration wallSleepTime = endWallTime - startWallTime;
  	double secs = wallSleepTime.toSec();

  	//check that the loop laster for the correct wall time
  	ASSERT_TRUE(secs > 1.98 && secs < 2.02);

  	//check that the ros end time is correct as time did not start at 0 seconds
    ASSERT_NEAR( 0.00001, wallSleepTime.toSec() * 5, endTime.toSec() - 1000);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "clock_server_node_test");

  return RUN_ALL_TESTS();
}