#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"
#include "vehicle_auto_control/DynamicLawnmowerAction.h"
#include "vehicle_auto_control/Velocity.h"

ros::ServiceClient client;


TEST(PointPath, PointPathController){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v1/command_target_velocity", 1000);    
    actionlib::SimpleActionClient<vehicle_auto_control::PointPathAction> ac("/vehicle_controller/v1/point_path", true);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::PointPathGoal pointPathMsg;

    maxVelMsg.horizontalVelocity = 1.0;
    maxVelMsg.verticalVelocity = 1.0;
    maxVelMsg.rotationalVelocity = 0.349066;

    geometry_msgs::Point point1;
    point1.x = 10;
    point1.y = -10;
    point1.z = -105;

    geometry_msgs::Point point2;
    point2.x = 25.5;
    point2.y = -15.5;
    point2.z = -95;

    pointPathMsg.points.push_back(point1);
    pointPathMsg.points.push_back(point2);
    pointPathMsg.yoyo = false;

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 200.0);

    if(targetVelPub.getNumSubscribers() == 0 || !ac.waitForServer(ros::Duration(5)))
    {
        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v1", ros::Time(0), ros::Duration(10.0));

    targetVelPub.publish(maxVelMsg);
    ac.sendGoal(pointPathMsg);
    

    unsigned currentPoint = 0;
    ros::Time start = ros::Time::now();

    bool isActive = false;
    while(currentPoint < pointPathMsg.points.size() && (ros::Time::now() - start) <= ros::Duration(400.0))
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

        if(zDistance <= 1.0 && xyDistance <= 5.0)
        {
            currentPoint++;
        }

        if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            isActive = true;
        }
    }
    
    bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

    ASSERT_EQ(true, finishedBeforeTimeout);
    ASSERT_EQ(true, isActive);
    ASSERT_EQ(actionlib::SimpleClientGoalState::SUCCEEDED, ac.getState().state_);
    ASSERT_EQ(pointPathMsg.points.size(), currentPoint);
}

TEST(PointPath, YoYoPointPathController){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    ros::Publisher pointPathPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v0/command_target_velocity", 1000);
    actionlib::SimpleActionClient<vehicle_auto_control::PointPathAction> ac("/vehicle_controller/v0/point_path", true);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::PointPathGoal pointPathMsg;

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
    pointPathMsg.yoyo = true;

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 5);

    if(targetVelPub.getNumSubscribers() == 0 || !ac.waitForServer(ros::Duration(10.0)))
    {
        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v0", ros::Time(0), ros::Duration(200.0));

    targetVelPub.publish(maxVelMsg);
  //  ros::Duration(10).sleep();
    ac.sendGoal(pointPathMsg);

    unsigned currentPoint = 0;

    bool goingUp = true;
    unsigned yoyo = 0;
    bool isActive = false;
    ros::Time start = ros::Time::now();
    while(currentPoint < pointPathMsg.points.size() && (ros::Time::now() - start) <= ros::Duration(400.0))
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

        if(xyDistance <= 5.0)
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
        
        if(zDistance <= 1.0)
        {
            goingUp = !goingUp;
            yoyo++;
        }

        if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            isActive = true;
        }
    }

    bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

    ASSERT_EQ(true, finishedBeforeTimeout);
    ASSERT_EQ(true, isActive);
    ASSERT_EQ(actionlib::SimpleClientGoalState::SUCCEEDED, ac.getState().state_);
    ASSERT_EQ(pointPathMsg.points.size(), currentPoint);
    ASSERT_TRUE(yoyo >= 4);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "point_path_test");

  return RUN_ALL_TESTS();
}
