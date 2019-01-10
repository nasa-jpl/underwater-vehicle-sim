#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Vector3.h"

#include "vehicles/FourDOFPropulsion.h"
#include "vehicles/VehicleState.h"

TEST(FourDOFPropulsion, SendCommand) {
    //Initalize ROS node handle
    ros::NodeHandle n("/underwater_vehicle_sim/vehicles/v1");
    VehicleState state(n);
    FourDOFPropulsion module("prop", state, n);

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/underwater_vehicle_sim/vehicles/v1/prop/command_velocity", 1000);

     //wait for subscriber, should be almost instant
    while(vel_pub.getNumSubscribers() <= 0);

    //This needs to be a pointer or else message will not reliably
    //publish with a single ros::spinOnce() with no wait.
    geometry_msgs::TwistPtr msg(new geometry_msgs::Twist);

    tf2::Vector3 linearVelocity(1, 2, -3);
    tf2::Vector3 angularVelocity(0.1, -0.2, 0.3);
    msg->linear.x = linearVelocity.x();
    msg->linear.y = linearVelocity.y();
    msg->linear.z = linearVelocity.z();

    msg->angular.x = angularVelocity.x();
    msg->angular.y = angularVelocity.y();
    msg->angular.z = angularVelocity.z();
    
    vel_pub.publish(msg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(linearVelocity.x(), state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(linearVelocity.y(), state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(linearVelocity.z(), state.getLinearVelocity().z());

    EXPECT_NEAR(0.0, state.getAngularVelocity().x(), 0.00000000001);
    EXPECT_NEAR(0.0, state.getAngularVelocity().y(), 0.00000000001);
    EXPECT_DOUBLE_EQ(angularVelocity.z(), state.getAngularVelocity().z());


    //Check that max velocities are followed
    geometry_msgs::TwistPtr msgMax(new geometry_msgs::Twist);

    tf2::Vector3 linearVelocityMax(6, 6, 6);
    tf2::Vector3 angularVelocityMax(0.1, -0.2, 0.5);
    msgMax->linear.x = linearVelocityMax.x();
    msgMax->linear.y = linearVelocityMax.y();
    msgMax->linear.z = linearVelocityMax.z();

    msgMax->angular.x = angularVelocityMax.x();
    msgMax->angular.y = angularVelocityMax.y();
    msgMax->angular.z = angularVelocityMax.z();
    
    vel_pub.publish(msgMax);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(5, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(5, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(4, state.getLinearVelocity().z());

    EXPECT_NEAR(0.0, state.getAngularVelocity().x(), 0.00000000001);
    EXPECT_NEAR(0.0, state.getAngularVelocity().y(), 0.00000000001);
    EXPECT_NEAR(0.4, state.getAngularVelocity().z(), 0.00000000001);

    //Check that min velocity values are followed
    geometry_msgs::TwistPtr msgMin(new geometry_msgs::Twist);

    tf2::Vector3 linearVelocityMin(-6, -6, -6);
    tf2::Vector3 angularVelocityMin(-0.1, 0.2, -0.5);
    msgMin->linear.x = linearVelocityMin.x();
    msgMin->linear.y = linearVelocityMin.y();
    msgMin->linear.z = linearVelocityMin.z();

    msgMin->angular.x = angularVelocityMin.x();
    msgMin->angular.y = angularVelocityMin.y();
    msgMin->angular.z = angularVelocityMin.z();
    
    vel_pub.publish(msgMin);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(-5, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(-5, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(-4, state.getLinearVelocity().z());

    EXPECT_NEAR(0.0, state.getAngularVelocity().x(), 0.00000000001);
    EXPECT_NEAR(0.0, state.getAngularVelocity().y(), 0.00000000001);
    EXPECT_NEAR(-0.4, state.getAngularVelocity().z(), 0.00000000001);
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
  ros::init(argc, argv, "four_dof_propulsion_test");
  
  broadcastStaticTransform();

  return RUN_ALL_TESTS();
}
