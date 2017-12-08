#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"

ros::ServiceClient client;

TEST(ClockServer, TestTime){
	//Initalize ROS node handle
  	ros::NodeHandle n;

  	//wait for the first non-zero time to start
  	while(ros::Time::now().toSec() < 0.000001);


  	ros::Time startTime = ros::Time::now();
  	ros::WallTime startWallTime = ros::WallTime::now();

  	//loop for 10 seconds in ros time
  	ros::Duration dur = (ros::Time::now() - startTime);
	while(dur.toSec() < 10.0)
  	{
  		dur = (ros::Time::now() - startTime);
  	}

  	ros::WallTime endWallTime = ros::WallTime::now();
  	ros::Time endTime = ros::Time::now();

  	ros::WallDuration wallSleepTime = endWallTime - startWallTime;
  	double secs = wallSleepTime.toSec();

  	//check that the loop laster for the correct wall time
  	ASSERT_TRUE(secs > 1.99 && secs < 2.01);

  	//check that the ros end time is correct as time did not start at 0 seconds
  	ASSERT_TRUE(endTime.toSec() > 1010 && endTime.toSec() < 1012);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "clock_server_test");

  return RUN_ALL_TESTS();
}