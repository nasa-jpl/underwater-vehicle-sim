#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include "vehicles/VehicleState.h"

ros::ServiceClient client;

bool doubleEqual(double d1, double d2)
{
    return fabs(d1 - d2) <= 0.00000001;
}

bool quaterionsEqual(tf2::Quaternion q1, tf2::Quaternion q2)
{    
    return doubleEqual(abs(q1.dot(q2)), 1);
}

TEST(VehicleState, FrameConvert){

    VehicleState state;
    
    std::vector<tf2::Vector3> posNED;
    std::vector<tf2::Vector3> posENU;

    std::vector<tf2::Quaternion> rotNED;
    std::vector<tf2::Quaternion> rotENU;

    posNED.push_back(tf2::Vector3(0,0,0));
    posENU.push_back(tf2::Vector3(0,0,0));

    posNED.push_back(tf2::Vector3(10,5,10));
    posENU.push_back(tf2::Vector3(5,10,-10));

    posNED.push_back(tf2::Vector3(-5.67,10.23,-10));
    posENU.push_back(tf2::Vector3(10.23,-5.67,10));


    tf2::Quaternion q1NED;
    q1NED.setRPY(0, 0, 0);
    tf2::Quaternion q1ENU;
    q1ENU.setRPY(M_PI, 0, M_PI / 2);
    rotNED.push_back(q1NED);
    rotENU.push_back(q1ENU);


    tf2::Quaternion q2NED;
    q2NED.setRPY(0, 0, M_PI / 2);
    tf2::Quaternion q2ENU;
    q2ENU.setRPY(M_PI, 0, 0);
    rotNED.push_back(q2NED);
    rotENU.push_back(q2ENU);


    tf2::Quaternion q3NED;
    q3NED.setRPY(0, 0, -M_PI / 2);
    tf2::Quaternion q3ENU;
    q3ENU.setRPY(M_PI, 0, M_PI);
    rotNED.push_back(q3NED);
    rotENU.push_back(q3ENU);


    tf2::Quaternion q4NED;
    q4NED.setRPY(0, M_PI / 2, 0);
    tf2::Quaternion q4ENU;
    q4ENU.setRPY(-M_PI / 2, -M_PI / 2, 0);
    rotNED.push_back(q4NED);
    rotENU.push_back(q4ENU);

    for(unsigned int i = 0; i < posNED.size(); i++)
    {
        state.setPositionNED(posNED[i]);
        EXPECT_NEAR(posNED[i].x(), state.getPositionNED().x(), 0.00000000001);
        EXPECT_NEAR(posNED[i].y(), state.getPositionNED().y(), 0.00000000001);
        EXPECT_NEAR(posNED[i].z(), state.getPositionNED().z(), 0.00000000001);

        EXPECT_NEAR(posENU[i].x(), state.getPositionENU().x(), 0.00000000001);
        EXPECT_NEAR(posENU[i].y(), state.getPositionENU().y(), 0.00000000001);
        EXPECT_NEAR(posENU[i].z(), state.getPositionENU().z(), 0.00000000001);

        state.setPositionENU(posENU[i]);
        EXPECT_NEAR(posNED[i].x(), state.getPositionNED().x(), 0.00000000001);
        EXPECT_NEAR(posNED[i].y(), state.getPositionNED().y(), 0.00000000001);
        EXPECT_NEAR(posNED[i].z(), state.getPositionNED().z(), 0.00000000001);

        EXPECT_NEAR(posENU[i].x(), state.getPositionENU().x(), 0.00000000001);
        EXPECT_NEAR(posENU[i].y(), state.getPositionENU().y(), 0.00000000001);
        EXPECT_NEAR(posENU[i].z(), state.getPositionENU().z(), 0.00000000001);
    }

    for(unsigned int i = 0; i < rotNED.size(); i++)
    {
        state.setRotationNED(rotNED[i]);
        EXPECT_TRUE(quaterionsEqual(rotNED[i], state.getRotationNED()));
        EXPECT_TRUE(quaterionsEqual(rotENU[i], state.getRotationENU()));

        state.setRotationENU(rotENU[i]);
        EXPECT_TRUE(quaterionsEqual(rotNED[i], state.getRotationNED()));
        EXPECT_TRUE(quaterionsEqual(rotENU[i], state.getRotationENU()));
    }
}

TEST(VehicleState, UpdateTest){
    //Initalize ROS node handle
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state;

    tf2::Vector3 linearVelocity(1, 2, 1);
    tf2::Vector3 angularVelocity(-0.2, -0.4, 0.6);

    tf2::Vector3 startPosition(1.2, 3.4, 5.6);
    tf2::Vector3 endPosition(0.2, 3.9, 6.1);

    tf2::Quaternion startRotation;
    startRotation.setRPY(0, 0, M_PI / 2);
    tf2::Quaternion rotation;
    rotation.setRPY(-0.2 * deltaTime.toSec(), 
                    -0.4 * deltaTime.toSec(), 
                    0.6 * deltaTime.toSec());
    tf2::Quaternion endRotation(startRotation * rotation);

    state.setPositionNED(startPosition);
    state.setRotationNED(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPositionNED());
    EXPECT_EQ(startRotation, state.getRotationNED());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);

    //After movement
    EXPECT_NEAR(endPosition.getX(), state.getPositionNED().getX(), 0.0000000001);
    EXPECT_NEAR(endPosition.getY(), state.getPositionNED().getY(), 0.0000000001);
    EXPECT_NEAR(endPosition.getZ(), state.getPositionNED().getZ(), 0.0000000001);

    EXPECT_TRUE(quaterionsEqual(endRotation, state.getRotationNED()));
}

TEST(VehicleState, UpperBoundTest){
        //Initalize ROS node handle
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state;

    tf2::Vector3 linearVelocity(0, 0, -1);
    tf2::Vector3 angularVelocity(0, 0, 0);

    tf2::Vector3 startPosition(0, 0, 0.1);
    tf2::Vector3 endPosition(0, 0, 0);

    tf2::Quaternion startRotation;
    startRotation.setRPY(0, 0, 0);
    state.setPositionNED(startPosition);
    state.setRotationNED(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPositionNED());
    EXPECT_EQ(startRotation, state.getRotationNED());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);

    //After movement
    EXPECT_NEAR(endPosition.getX(), state.getPositionNED().getX(), 0.0000000001);
    EXPECT_NEAR(endPosition.getY(), state.getPositionNED().getY(), 0.0000000001);
    EXPECT_NEAR(endPosition.getZ(), state.getPositionNED().getZ(), 0.0000000001);
}

TEST(VehicleState, LowerBoundTest){
    //Initalize ROS node handle
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state;

    tf2::Vector3 linearVelocity(0, 0, 1);
    tf2::Vector3 angularVelocity(0, 0, 0);

    tf2::Vector3 startPosition(0, 0, 199.7);
    tf2::Vector3 endPosition(0, 0, 199.9); //placed 0.1 meters above the seafloor 

    tf2::Quaternion startRotation;
    startRotation.setRPY(0, 0, 0);
    
    state.setPositionNED(startPosition);
    state.setRotationNED(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPositionNED());
    EXPECT_EQ(startRotation, state.getRotationNED());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.updatePose(currentTime, deltaTime);
    ModelData data;
    data.depth = 200;
    state.seafloorCollision(data);

    //After movement
    EXPECT_NEAR(endPosition.getX(), state.getPositionNED().getX(), 0.00000000001);
    EXPECT_NEAR(endPosition.getY(), state.getPositionNED().getY(), 0.00000000001);
    EXPECT_NEAR(endPosition.getZ(), state.getPositionNED().getZ(), 0.00000000001);
}

TEST(VehicleState, EvectByCurrentsTest)
{
    //Initalize ROS node handle
    ros::Time currentTime(100);
    ros::Duration deltaTime(0.5);

    VehicleState state;

    tf2::Vector3 linearVelocity(0.26, -0.16, 1);
    tf2::Vector3 angularVelocity(0, 0, 0);

    ModelData data;
    data.u = -0.04; //eastward
    data.v = 0.12; //northward
    data.w = 0.5; //upward

    tf2::Vector3 startPosition(0, 0, 20);
    tf2::Vector3 endPosition(0.06, -0.02, 19.75);//NED (northward, eastward, downward)

    tf2::Quaternion startRotation;
    startRotation.setRPY(0, 0, 0);
    
    state.setPositionNED(startPosition);
    state.setRotationNED(startRotation);
    state.setLinearVelocity(linearVelocity);
    state.setAngularVelocity(angularVelocity);

    //Before movement
    EXPECT_EQ(startPosition, state.getPositionNED());
    EXPECT_EQ(startRotation, state.getRotationNED());
    EXPECT_EQ(linearVelocity, state.getLinearVelocity());
    EXPECT_EQ(angularVelocity, state.getAngularVelocity());

    state.evectByCurrents(data, deltaTime);

    //After movement
    EXPECT_NEAR(endPosition.getX(), state.getPositionNED().getX(), 0.00000000001);
    EXPECT_NEAR(endPosition.getY(), state.getPositionNED().getY(), 0.00000000001);
    EXPECT_NEAR(endPosition.getZ(), state.getPositionNED().getZ(), 0.00000000001);
}

//Had issues doing this in the roslaunch file for this test. Not sure why.
//Normally this can be included in the roslaunch file with the following
//<node pkg="tf2_ros" type="static_transform_publisher" name="ned_publisher" args="0 0 0 1.57 0 3.14 world world_ned"/>
void broadcastStaticTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "world_ned";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, M_PI / 2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "vehicle_state_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
