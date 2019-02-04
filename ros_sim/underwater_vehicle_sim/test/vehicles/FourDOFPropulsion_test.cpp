#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Vector3.h"

#include "std_msgs/Float64.h"

#include "vehicles/FourDOFPropulsion.h"
#include "vehicles/VehicleState.h"

TEST(FourDOFPropulsion, SendCommand) {
    //Initalize ROS node handle
    ros::NodeHandle n;
    VehicleState state;
    FourDOFPropulsion module(state);

    ros::Publisher forward_thrust_pub = n.advertise<std_msgs::Float64>("/v1/command_forward_thruster", 1000);
    ros::Publisher lateral_thrust_pub = n.advertise<std_msgs::Float64>("/v1/command_lateral_thruster", 1000);
    ros::Publisher vertical_thrust_pub = n.advertise<std_msgs::Float64>("/v1/command_vertical_thruster", 1000);
    ros::Publisher rudder_pub = n.advertise<std_msgs::Float64>("/v1/command_rudder", 1000);

    //wait for subscribers, should be almost instant
    while(forward_thrust_pub.getNumSubscribers() <= 0);
    while(lateral_thrust_pub.getNumSubscribers() <= 0);
    while(vertical_thrust_pub.getNumSubscribers() <= 0);
    while(rudder_pub.getNumSubscribers() <= 0);

    
    //This needs to be a pointer or else message will not reliably
    //publish with a single ros::spinOnce() with no wait.
    std_msgs::Float64Ptr forwardMsg(new std_msgs::Float64);
    forwardMsg->data = 50;

    std_msgs::Float64Ptr lateralMsg(new std_msgs::Float64);
    lateralMsg->data = -25;

    std_msgs::Float64Ptr verticalMsg(new std_msgs::Float64);
    verticalMsg->data = 100;

    std_msgs::Float64Ptr rudderMsg(new std_msgs::Float64);
    rudderMsg->data = -45;

    //Check forward thrust
    forward_thrust_pub.publish(forwardMsg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(0, state.getLinearVelocity().z());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().y());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().z());


    //Check lateral thrust
    lateral_thrust_pub.publish(lateralMsg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(-0.25, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(0, state.getLinearVelocity().z());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().y());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().z());


    //Check vertical thrust
    vertical_thrust_pub.publish(verticalMsg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(-0.25, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().z());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().y());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().z());


    //Check rudder
    rudder_pub.publish(rudderMsg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(-0.25, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().z());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().y());
    EXPECT_DOUBLE_EQ(-5, state.getAngularVelocity().z());



    //Test invalid messages
    std_msgs::Float64Ptr forwardMsgInvalid(new std_msgs::Float64);
    forwardMsg->data = 101;

    std_msgs::Float64Ptr lateralMsgInvalid(new std_msgs::Float64);
    lateralMsg->data = -101;

    std_msgs::Float64Ptr verticalMsgInvalid(new std_msgs::Float64);
    verticalMsg->data = 101;

    std_msgs::Float64Ptr rudderMsgInvalid(new std_msgs::Float64);
    rudderMsg->data = -46;

    forward_thrust_pub.publish(forwardMsg);    
    lateral_thrust_pub.publish(lateralMsg);    
    vertical_thrust_pub.publish(verticalMsg);    
    rudder_pub.publish(rudderMsg);    
    ros::spinOnce();

    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().x());
    EXPECT_DOUBLE_EQ(-0.25, state.getLinearVelocity().y());
    EXPECT_DOUBLE_EQ(1, state.getLinearVelocity().z());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().x());
    EXPECT_DOUBLE_EQ(0, state.getAngularVelocity().y());
    EXPECT_DOUBLE_EQ(-5, state.getAngularVelocity().z());
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
