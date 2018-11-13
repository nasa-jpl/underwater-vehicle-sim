#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"

ros::ServiceClient client;
/*
TEST(FourDOFPropulsion, TestVehicleLinearMovement){
        //Initalize ROS node handle
        ros::NodeHandle n;

        tf::TransformListener transformListener;
        ros::Publisher velocityV1Pub = n.advertise<geometry_msgs::Twist>("/vehicles/v1/prop_v1/command_velocity", 1000);
        
        tf::StampedTransform transformV1Start;

        tf::StampedTransform transformV1End;

        bool transformsRecieved = false;

        while(!transformsRecieved)
        {
            try
            {
                transformListener.lookupTransform("/world", "/v1",  
                                          ros::Time(0), transformV1Start);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        geometry_msgs::Twist msg;
        geometry_msgs::Vector3 lin;
        geometry_msgs::Vector3 rot;

        lin.x = 1.5; //max set to 1.5
        lin.y = -0.5; //max set to 0.5
        lin.z = -0.25; //max set to 0.25

        rot.x = 0;
        rot.y = 0;
        rot.z = 0.0; //max set to 0

        msg.linear = lin;
        msg.angular = rot;

        velocityV1Pub.publish(msg);
        ros::spinOnce();

        ros::Time startSleep = ros::Time::now();
        ros::Duration(2).sleep();
        try
            {
                transformListener.lookupTransform("/world", "/v1",  
                                          ros::Time(0), transformV1End);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

        float sleepDuration = (transformV1End.stamp_ - startSleep).toSec();

        //Due to timing issues they are not exact, however they are acceptably close
        bool xCorrect = (1.5 * sleepDuration) + 0.05 > (transformV1End.getOrigin().getX() - transformV1Start.getOrigin().getX()) &&
                        (1.5 * sleepDuration) - 0.05 < (transformV1End.getOrigin().getX() - transformV1Start.getOrigin().getX());

        bool yCorrect = (-0.5 * sleepDuration) + 0.05 > (transformV1End.getOrigin().getY() - transformV1Start.getOrigin().getY()) &&
                        (-0.5 * sleepDuration) - 0.05 < (transformV1End.getOrigin().getY() - transformV1Start.getOrigin().getY());

        bool zCorrect = (-0.25 * sleepDuration) + 0.05 > (transformV1End.getOrigin().getZ() - transformV1Start.getOrigin().getZ()) &&
                        (-0.25 * sleepDuration) - 0.05 < (transformV1End.getOrigin().getZ() - transformV1Start.getOrigin().getZ());
         
        ASSERT_TRUE(xCorrect);
        ASSERT_TRUE(yCorrect);
        ASSERT_TRUE(zCorrect);
}

TEST(FourDOFPropulsion, HertzTest){
    //Initalize ROS node handle
    ros::NodeHandle n;

    tf::TransformListener transformListener;
    ros::Publisher velocityV1Pub = n.advertise<geometry_msgs::Twist>("/vehicles/v0/prop_v1/command_velocity", 1000);
    
    tf::StampedTransform transformV1Start;

    tf::StampedTransform transformV1Update;

    bool transformsRecieved = false;

    while(!transformsRecieved)
    {
        try
        {
            transformListener.lookupTransform("/world", "/v0",  
                                      ros::Time(0), transformV1Start);
            transformsRecieved = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    geometry_msgs::Twist msg;
    geometry_msgs::Vector3 lin;
    geometry_msgs::Vector3 rot;

    lin.x = 1.5;
    lin.y = -0.5;
    lin.z = -0.25;

    rot.x = 0;
    rot.y = 0;
    rot.z = 0;

    msg.linear = lin;
    msg.angular = rot;

    velocityV1Pub.publish(msg);


    ros::Time start = ros::Time::now();
    bool firstRecieved = false;
    while(ros::Time::now() - start < ros::Duration(2))
    {
        try
        {
            transformListener.lookupTransform("/world", "/v0",  
                                      ros::Time(0), transformV1Update);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        if(transformV1Update.stamp_ != transformV1Start.stamp_)
        {
            if(firstRecieved)
            {
                ASSERT_NEAR(0.25, (transformV1Update.stamp_ - transformV1Start.stamp_).toSec(), 0.05);
                transformV1Start = transformV1Update;
            }
            else
            {
                firstRecieved = true;
            }
        }
    }
}

TEST(FourDOFPropulsion, TestVehicleRotationalMovement) {
        //Initalize ROS node handle
        ros::NodeHandle n;

        tf::TransformListener transformListener;
        ros::Publisher velocityV1Pub = n.advertise<geometry_msgs::Twist>("/vehicles/v2/prop_v2/command_velocity", 1000);
        
        tf::StampedTransform transformV2Start;

        tf::StampedTransform transformV2End;

        bool transformsRecieved = false;

        while(!transformsRecieved)
        {
            try
            {
                transformListener.lookupTransform("/world", "/v2",  
                                          ros::Time(0), transformV2Start);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        geometry_msgs::Twist msg;
        geometry_msgs::Vector3 lin;
        geometry_msgs::Vector3 rot;

        lin.x = 0;
        lin.y = 0;
        lin.z = 0;

        rot.x = 0;
        rot.y = 0;
        rot.z = -0.125;

        msg.linear = lin;
        msg.angular = rot;

        velocityV1Pub.publish(msg);
        ros::spinOnce();

        ros::Time startSleep = ros::Time::now();
        ros::Duration(2).sleep();
        try
            {
                transformListener.lookupTransform("/world", "/v2",  
                                          ros::Time(0), transformV2End);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

        float sleepDuration = (transformV2End.stamp_ - startSleep).toSec();

        //Due to timing issues they are not exact, however they are acceptably close

        tf::Vector3 axis = transformV2End.getRotation().getAxis();

        bool angleCorrect = (-0.125 * sleepDuration * axis.getZ()) + 0.02 > (transformV2End.getRotation().getAngle() - transformV2Start.getRotation().getAngle()) &&
                        (-0.125 * sleepDuration * axis.getZ()) - 0.02 < (transformV2End.getRotation().getAngle() - transformV2Start.getRotation().getAngle());
         
        
        ASSERT_FLOAT_EQ(0, axis.getX());
        ASSERT_FLOAT_EQ(0, axis.getY());
        ASSERT_FLOAT_EQ(-1, axis.getZ());
        ASSERT_TRUE(angleCorrect);
}

TEST(FourDOFPropulsion, TestVehicleRotationalThenLinearMovement) {
        //Initalize ROS node handle
        ros::NodeHandle n;

        tf::TransformListener transformListener;
        ros::Publisher velocityV3Pub = n.advertise<geometry_msgs::Twist>("/vehicles/v3/prop_v3/command_velocity", 1000);
        
        tf::StampedTransform transformV3Start;

        tf::StampedTransform transformV3End;

        bool transformsRecieved = false;

        while(!transformsRecieved)
        {
            try
            {
                transformListener.lookupTransform("/world", "/v3",  
                                          ros::Time(0), transformV3Start);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        geometry_msgs::Twist rotMsg;
        geometry_msgs::Vector3 rotLin;
        geometry_msgs::Vector3 rotRot;
        rotLin.x = 0;
        rotLin.y = 0;
        rotLin.z = 0;
        rotRot.x = 0;
        rotRot.y = 0;
        rotRot.z = -M_PI / 4;
        rotMsg.linear = rotLin;
        rotMsg.angular = rotRot;


        geometry_msgs::Twist linMsg;
        geometry_msgs::Vector3 linLin;
        geometry_msgs::Vector3 linRot;
        linLin.x = 1.5;
        linLin.y = 0;
        linLin.z = 0;
        linRot.x = 0;
        linRot.y = 0;
        linRot.z = 0;
        linMsg.linear = linLin;
        linMsg.angular = linRot;

        velocityV3Pub.publish(rotMsg);
        ros::spinOnce();
        ros::Duration(2).sleep();


        velocityV3Pub.publish(linMsg);
        ros::spinOnce();

        ros::Time startSleep = ros::Time::now();
        ros::Duration(2).sleep();


        try
        {
            transformListener.lookupTransform("/world", "/v3",  
                                      ros::Time(0), transformV3End);
            transformsRecieved = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


        float sleepDuration = (transformV3End.stamp_ - startSleep).toSec();

        //Due to timing issues they are not exact, however they are acceptably close

        //Due to timing issues they are not exact, however they are acceptably close
        bool xCorrect = (0 * sleepDuration) + 0.05 > (transformV3End.getOrigin().getX() - transformV3Start.getOrigin().getX()) &&
                        (0 * sleepDuration) - 0.05 < (transformV3End.getOrigin().getX() - transformV3Start.getOrigin().getX());

        bool yCorrect = (-1.5 * sleepDuration) + 0.05 > (transformV3End.getOrigin().getY() - transformV3Start.getOrigin().getY()) &&
                        (-1.5 * sleepDuration) - 0.05 < (transformV3End.getOrigin().getY() - transformV3Start.getOrigin().getY());

        bool zCorrect = (0 * sleepDuration) + 0.05 > (transformV3End.getOrigin().getZ() - transformV3Start.getOrigin().getZ()) &&
                        (0 * sleepDuration) - 0.05 < (transformV3End.getOrigin().getZ() - transformV3Start.getOrigin().getZ());
        
        ASSERT_TRUE(xCorrect);
        ASSERT_TRUE(yCorrect);
        ASSERT_TRUE(zCorrect);
}
*/
TEST(FourDOFPropulsion, TestSeafloorImpact) {
        //Initalize ROS node handle
        ros::NodeHandle n;

        tf::TransformListener transformListener;
        ros::Publisher velocityV4Pub = n.advertise<geometry_msgs::Twist>("/vehicles/v4/prop_v4/command_velocity", 1000);
        
        tf::StampedTransform transformV4Start;

        tf::StampedTransform transformV4End;

        bool transformsRecieved = false;

        while(!transformsRecieved)
        {
            try
            {
                transformListener.lookupTransform("/world", "/v4",  
                                          ros::Time(0), transformV4Start);
                transformsRecieved = true;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        geometry_msgs::Twist linMsg;
        geometry_msgs::Vector3 linLin;
        geometry_msgs::Vector3 linRot;
        linLin.x = 0;
        linLin.y = 0;
        linLin.z = -2;
        linRot.x = 0;
        linRot.y = 0;
        linRot.z = 0;
        linMsg.linear = linLin;
        linMsg.angular = linRot;

        velocityV4Pub.publish(linMsg);
        ros::spinOnce();

        ros::Duration(5).sleep();


        try
        {
            transformListener.lookupTransform("/world", "/v4",  
                                      ros::Time(0), transformV4End);
            transformsRecieved = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


        ASSERT_NEAR(-200, transformV4End.getOrigin().getZ(), 0.1);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "four_dof_propulsion_test");

  return RUN_ALL_TESTS();
}
