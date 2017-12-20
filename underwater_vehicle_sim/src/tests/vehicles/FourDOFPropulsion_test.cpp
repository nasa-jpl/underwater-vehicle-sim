#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"
#include "tf/transform_listener.h"

ros::ServiceClient client;

TEST(InitVehicles, InitVehiclesLocationTest){
        //Initalize ROS node handle
        ros::NodeHandle n;

        tf::TransformListener transformListener;
/*        
        tf::StampedTransform transformV1;
        tf::StampedTransform transformV2;
        tf::StampedTransform transformV3;

        bool transformsRecieved = false;

        while(!transformsRecieved)
        {
			try
			{
		  		transformListener.lookupTransform("/world", "/v1",  
		                                  ros::Time(0), transformV1);
		  		transformListener.lookupTransform("/world", "/v2",  
		                                  ros::Time(0), transformV2);
		  		transformListener.lookupTransform("/world", "/v3",  
		                                  ros::Time(0), transformV3);
		  		transformsRecieved = true;
		    }
		    catch (tf::TransformException ex)
		    {
		    	ROS_ERROR("%s",ex.what());
		        ros::Duration(1.0).sleep();
		    }
		}


        ASSERT_FLOAT_EQ(-100, transformV1.getOrigin().getX());
        ASSERT_FLOAT_EQ(100, transformV1.getOrigin().getY());
        ASSERT_FLOAT_EQ(-100, transformV1.getOrigin().getZ());

        ASSERT_FLOAT_EQ(521.25, transformV2.getOrigin().getX());
        ASSERT_FLOAT_EQ(-434.5, transformV2.getOrigin().getY());
        ASSERT_FLOAT_EQ(0, transformV2.getOrigin().getZ());

        ASSERT_FLOAT_EQ(200, transformV3.getOrigin().getX());
        ASSERT_FLOAT_EQ(-100, transformV3.getOrigin().getY());
        ASSERT_FLOAT_EQ(-234, transformV3.getOrigin().getZ());
*/}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "four_dof_propulsion_test");

  return RUN_ALL_TESTS();
}
