#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"
#include "vehicles/VehicleState.h"
#include "tf/transform_listener.h"

ros::ServiceClient client;

bool doubleEqual(double d1, double d2)
{
    return fabs(d1 - d2) <= 0.00000001;
}

bool quaterionsEqual(tf::Quaternion q1, tf::Quaternion q2)
{    
    bool equal1 = doubleEqual(q1.getAxis().getX(), q2.getAxis().getX()) &&
                  doubleEqual(q1.getAxis().getY(), q2.getAxis().getY()) &&
                  doubleEqual(q1.getAxis().getZ(), q2.getAxis().getZ()) &&
                  doubleEqual(q1.getW(), q2.getW());

    tf::Quaternion q2Inverse = q2.inverse();
    bool equal2 = doubleEqual(q1.getAxis().getX(), q2Inverse.getAxis().getX()) &&
                  doubleEqual(q1.getAxis().getY(), q2Inverse.getAxis().getY()) &&
                  doubleEqual(q1.getAxis().getZ(), q2Inverse.getAxis().getZ()) &&
                  doubleEqual(q1.getW(), q2Inverse.getW());
    return equal1 || equal2;
}

TEST(VehicleState, UpdateTest){
    //Initalize ROS node handle
    ros::NodeHandle n;
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state(n);

    tf::Vector3 linearVelocity(1, 2, -1);
    tf::Vector3 angularVelocity(-0.2, -0.4, 0.6);

    tf::Vector3 startPosition(1.2, 3.4, -5.6);
    tf::Vector3 endPosition(0.2, 3.9, -6.1);

    tf::Quaternion startRotation;
    startRotation.setRPY(0, 0, M_PI / 2);
    tf::Quaternion rotation;
    rotation.setRPY(-0.2 * deltaTime.toSec(), 
                    -0.4 * deltaTime.toSec(), 
                    0.6 * deltaTime.toSec());
    tf::Quaternion endRotation(startRotation * rotation);

    state.setPosition(startPosition);
    state.setRotation(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPosition());
    EXPECT_EQ(startRotation, state.getRotation());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);

    //After movement
    EXPECT_DOUBLE_EQ(endPosition.getX(), state.getPosition().getX());
    EXPECT_DOUBLE_EQ(endPosition.getY(), state.getPosition().getY());
    EXPECT_DOUBLE_EQ(endPosition.getZ(), state.getPosition().getZ());

    EXPECT_TRUE(quaterionsEqual(endRotation, state.getRotation()));
}

TEST(VehicleState, UpperBoundTest){
        //Initalize ROS node handle
    ros::NodeHandle n;
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state(n);

    tf::Vector3 linearVelocity(0, 0, 1);
    tf::Vector3 angularVelocity(0, 0, 0);

    tf::Vector3 startPosition(0, 0, -0.1);
    tf::Vector3 endPosition(0, 0, 0);

    tf::Quaternion startRotation;
    startRotation.setRPY(0, 0, 0);
    state.setPosition(startPosition);
    state.setRotation(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPosition());
    EXPECT_EQ(startRotation, state.getRotation());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);

    //After movement
    EXPECT_DOUBLE_EQ(endPosition.getX(), state.getPosition().getX());
    EXPECT_DOUBLE_EQ(endPosition.getY(), state.getPosition().getY());
    EXPECT_DOUBLE_EQ(endPosition.getZ(), state.getPosition().getZ());
}

TEST(VehicleState, LowerBoundTest){
        //Initalize ROS node handle
    ros::NodeHandle n;
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state(n);

    tf::Vector3 linearVelocity(0, 0, -1);
    tf::Vector3 angularVelocity(0, 0, 0);

    tf::Vector3 startPosition(0, 0, -199.7);
    tf::Vector3 endPosition(0, 0, -199.9); //placed 0.1 meters above the seafloor 

    tf::Quaternion startRotation;
    startRotation.setRPY(0, 0, 0);
    
    state.setPosition(startPosition);
    state.setRotation(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPosition());
    EXPECT_EQ(startRotation, state.getRotation());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);

    //After movement
    EXPECT_DOUBLE_EQ(endPosition.getX(), state.getPosition().getX());
    EXPECT_DOUBLE_EQ(endPosition.getY(), state.getPosition().getY());
    EXPECT_DOUBLE_EQ(endPosition.getZ(), state.getPosition().getZ());
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "vehicle_state_test");

  return RUN_ALL_TESTS();
}
