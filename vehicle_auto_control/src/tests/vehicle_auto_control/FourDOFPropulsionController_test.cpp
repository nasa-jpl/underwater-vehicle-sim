#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "vehicle_auto_control/PointPath.h"
#include "vehicle_auto_control/YoYoPointPath.h"
#include "vehicle_auto_control/Velocity.h"

ros::ServiceClient client;

TEST(FourDOFPropulsionController, PointPathController){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    ros::Publisher pointPathPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v1/command_target_velocity", 1000);
    pointPathPub = nh.advertise<vehicle_auto_control::PointPath>("/vehicle_controller/v1/command_point_path", 1000);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::PointPath pointPathMsg;

    maxVelMsg.horizontalVelocity = 1.0;
    maxVelMsg.verticalVelocity = 1.0;
    maxVelMsg.rotationalVelocity = 0.349066;

    geometry_msgs::Point point1;
    point1.x = 10;
    point1.y = -10;
    point1.z = -105;

    geometry_msgs::Point point2;
    point2.x = 5.5;
    point2.y = -10.5;
    point2.z = -95;

    pointPathMsg.points.push_back(point1);
    pointPathMsg.points.push_back(point2);

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0 || pointPathPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 5);

    if(targetVelPub.getNumSubscribers() == 0 || pointPathPub.getNumSubscribers() == 0)
    {
        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v1", ros::Time(0), ros::Duration(200.0));

    targetVelPub.publish(maxVelMsg);
    pointPathPub.publish(pointPathMsg);
    
    unsigned currentPoint = 0;

    ros::Time start = ros::Time::now();
    while(currentPoint < pointPathMsg.points.size() && (ros::Time::now() - start) <= ros::Duration(200.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v1",  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            FAIL();
        }

        double xDistance = fabs(transform.getOrigin().getX() - pointPathMsg.points[currentPoint].x);
        double yDistance = fabs(transform.getOrigin().getY() - pointPathMsg.points[currentPoint].y);
        double zDistance = fabs(transform.getOrigin().getZ() - pointPathMsg.points[currentPoint].z);
        double xyDistance = sqrt(xDistance * xDistance + yDistance * yDistance);

        if(zDistance <= 0.25 && xyDistance <= 1.0)
        {
            currentPoint++;
        }
    }
    ASSERT_EQ(pointPathMsg.points.size(), currentPoint);
}

TEST(FourDOFPropulsionController, YoYoPointPathController){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    ros::Publisher pointPathPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v0/command_target_velocity", 1000);
    pointPathPub = nh.advertise<vehicle_auto_control::YoYoPointPath>("/vehicle_controller/v0/command_yoyo_point_path", 1000);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::YoYoPointPath pointPathMsg;

    maxVelMsg.horizontalVelocity = 1.0;
    maxVelMsg.verticalVelocity = 1.0;
    maxVelMsg.rotationalVelocity = 0.349066;

    geometry_msgs::Point point1;
    point1.x = 20;
    point1.y = -20;
    point1.z = -100;

    geometry_msgs::Point point2;
    point2.x = 20;
    point2.y = 0;
    point2.z = -100;

    pointPathMsg.points.push_back(point1);
    pointPathMsg.points.push_back(point2);
    pointPathMsg.upperDepth = -95;
    pointPathMsg.lowerDepth = -105;

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0 || pointPathPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 5);

    if(targetVelPub.getNumSubscribers() == 0 || pointPathPub.getNumSubscribers() == 0)
    {
        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v0", ros::Time(0), ros::Duration(200.0));

    targetVelPub.publish(maxVelMsg);
    pointPathPub.publish(pointPathMsg);
    
    unsigned currentPoint = 0;

    bool goingUp = true;
    unsigned yoyo = 0;
    ros::Time start = ros::Time::now();
    while(currentPoint < pointPathMsg.points.size() && (ros::Time::now() - start) <= ros::Duration(200.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v0",  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            FAIL();
        }

        double xDistance = fabs(transform.getOrigin().getX() - pointPathMsg.points[currentPoint].x);
        double yDistance = fabs(transform.getOrigin().getY() - pointPathMsg.points[currentPoint].y);
        
        double xyDistance = sqrt(xDistance * xDistance + yDistance * yDistance);

        if(xyDistance <= 1.0)
        {
            currentPoint++;
        }

        double zDistance = 0;
        if(goingUp)
        {
            zDistance = fabs(transform.getOrigin().getZ() - pointPathMsg.upperDepth);
        }
        else
        {
             zDistance = fabs(transform.getOrigin().getZ() - pointPathMsg.lowerDepth);
        }
        
        if(zDistance <= 0.25)
        {
            goingUp = !goingUp;
            yoyo++;
        }
    }
    ASSERT_EQ(pointPathMsg.points.size(), currentPoint);
    ASSERT_TRUE(yoyo >= 4);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "four_dof_propulsion_controller_test");

  return RUN_ALL_TESTS();
}
