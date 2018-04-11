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

TEST(DynamicLawnmower, DynamicLawnmowerTurn){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v2/command_target_velocity", 1000);    
    actionlib::SimpleActionClient<vehicle_auto_control::DynamicLawnmowerAction> ac("/vehicle_controller/v2/dynamic_lawnmower", true);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::DynamicLawnmowerGoal dynamicLawnmowerMsg;

    maxVelMsg.horizontalVelocity = 3.0;
    maxVelMsg.verticalVelocity = 1.0;
    maxVelMsg.rotationalVelocity = 0.349066;

    dynamicLawnmowerMsg.startLocation.x = 0;
    dynamicLawnmowerMsg.startLocation.y = 0;
    dynamicLawnmowerMsg.startLocation.z = -100;
    dynamicLawnmowerMsg.currentTrack = 0;
    dynamicLawnmowerMsg.currentSection = 0;
    dynamicLawnmowerMsg.alongTrackDirection = 0;
    dynamicLawnmowerMsg.acrossTrackDirection = M_PI / 2;
    dynamicLawnmowerMsg.trackSpacing = 10;
    dynamicLawnmowerMsg.targetHeight = -100;
    dynamicLawnmowerMsg.minSectionsPerTrack = 3;
    dynamicLawnmowerMsg.continueThreshold = 0.15;
    dynamicLawnmowerMsg.trackSectionThreshold = 2;

    std::vector<tf::Vector3> expectedPoints;

    expectedPoints.push_back(tf::Vector3(0,0,-100));
    expectedPoints.push_back(tf::Vector3(10,0,-100));
    expectedPoints.push_back(tf::Vector3(20,0,-100));
    expectedPoints.push_back(tf::Vector3(30,0,-100));
    expectedPoints.push_back(tf::Vector3(40,0,-100));
    expectedPoints.push_back(tf::Vector3(50,0,-100));
    expectedPoints.push_back(tf::Vector3(60,0,-100));
    expectedPoints.push_back(tf::Vector3(50,10,-100));
    expectedPoints.push_back(tf::Vector3(40,10,-100));
    expectedPoints.push_back(tf::Vector3(30,10,-100));
    expectedPoints.push_back(tf::Vector3(20,10,-100));
    expectedPoints.push_back(tf::Vector3(10,10,-100));
    expectedPoints.push_back(tf::Vector3(0,10,-100));
    expectedPoints.push_back(tf::Vector3(0,20,-100));

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Time startWait = ros::Time::now();
    while(targetVelPub.getNumSubscribers() == 0);

    if(targetVelPub.getNumSubscribers() == 0 || !ac.waitForServer(ros::Duration(5)))
    {
        FAIL();
    }
    //Wait for transforms
    listener.waitForTransform("/world", "/v2", ros::Time(0), ros::Duration(10.0));
    targetVelPub.publish(maxVelMsg);
    ac.sendGoal(dynamicLawnmowerMsg);

    ros::Time start = ros::Time::now();


    bool isActive = false;
    int currentPoint = 0;
    while(currentPoint < expectedPoints.size() && (ros::Time::now() - start) <= ros::Duration(500.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v2",  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            FAIL();
        }

        double xDistance = fabs(transform.getOrigin().getX() - expectedPoints[currentPoint].getX());
        double yDistance = fabs(transform.getOrigin().getY() - expectedPoints[currentPoint].getY());
        double zDistance = fabs(transform.getOrigin().getZ() - expectedPoints[currentPoint].getZ());
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
    ASSERT_EQ(expectedPoints.size(), currentPoint);    
}

TEST(DynamicLawnmower, DynamicLawnmowerStop){

    ros::NodeHandle nh;

    ros::Publisher targetVelPub;
    tf::TransformListener listener;

    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v3/command_target_velocity", 1000);    
    actionlib::SimpleActionClient<vehicle_auto_control::DynamicLawnmowerAction> ac("/vehicle_controller/v3/dynamic_lawnmower", true);

    vehicle_auto_control::Velocity maxVelMsg;
    vehicle_auto_control::DynamicLawnmowerGoal dynamicLawnmowerMsg;

    maxVelMsg.horizontalVelocity = 3.0;
    maxVelMsg.verticalVelocity = 1.0;
    maxVelMsg.rotationalVelocity = 0.349066;

    dynamicLawnmowerMsg.startLocation.x = 100;
    dynamicLawnmowerMsg.startLocation.y = 100;
    dynamicLawnmowerMsg.startLocation.z = -100;
    dynamicLawnmowerMsg.currentTrack = 0;
    dynamicLawnmowerMsg.currentSection = 0;
    dynamicLawnmowerMsg.alongTrackDirection = 0;
    dynamicLawnmowerMsg.acrossTrackDirection =  M_PI / 2;
    dynamicLawnmowerMsg.trackSpacing = 10;
    dynamicLawnmowerMsg.targetHeight = -100;
    dynamicLawnmowerMsg.minSectionsPerTrack = 3;
    dynamicLawnmowerMsg.continueThreshold = 0.15;
    dynamicLawnmowerMsg.trackSectionThreshold = 2;

    std::vector<tf::Vector3> expectedPoints;

    expectedPoints.push_back(tf::Vector3(100,100,-100));
    expectedPoints.push_back(tf::Vector3(110,100,-100));
    expectedPoints.push_back(tf::Vector3(120,100,-100));
    expectedPoints.push_back(tf::Vector3(130,100,-100));

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
    listener.waitForTransform("/world", "/v3", ros::Time(0), ros::Duration(10.0));

    targetVelPub.publish(maxVelMsg);
    ac.sendGoal(dynamicLawnmowerMsg);
    

    ros::Time start = ros::Time::now();

    bool isActive = false;
    int currentPoint = 0;
    while(currentPoint < expectedPoints.size() && (ros::Time::now() - start) <= ros::Duration(400.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v3",  
                                     ros::Time(0), transform);

            double xDistance = fabs(transform.getOrigin().getX() - expectedPoints[currentPoint].getX());
            double yDistance = fabs(transform.getOrigin().getY() - expectedPoints[currentPoint].getY());
            double zDistance = fabs(transform.getOrigin().getZ() - expectedPoints[currentPoint].getZ());
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
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        
    }

    bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

    ASSERT_TRUE(finishedBeforeTimeout);
    ASSERT_EQ(expectedPoints.size(), currentPoint);    
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dynamic_lawnmower_test");

  return RUN_ALL_TESTS();
}
