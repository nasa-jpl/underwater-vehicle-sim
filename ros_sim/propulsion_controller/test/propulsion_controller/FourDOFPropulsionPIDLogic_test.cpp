#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include "underwater_autonomy/util/VehiclePose.h"
#include "underwater_vehicle_msgs/VehicleData.h"


#include "propulsion_controller/FourDOFPropulsionPIDLogic.h"

using namespace underwater_autonomy;

std::vector<std_msgs::Bool> forwardEnables;
std::vector<std_msgs::Bool> lateralEnables;
std::vector<std_msgs::Bool> verticalEnables;
std::vector<std_msgs::Bool> rudderEnables;

std::vector<std_msgs::Float64> forwardStates;
std::vector<std_msgs::Float64> forwardSetpoints;

std::vector<std_msgs::Float64> lateralStates;
std::vector<std_msgs::Float64> lateralSetpoints;

std::vector<std_msgs::Float64> verticalStates;
std::vector<std_msgs::Float64> verticalSetpoints;

std::vector<std_msgs::Float64> rudderStates;
std::vector<std_msgs::Float64> rudderSetpoints;

std::vector<std_msgs::Float64> forwardCommand;
std::vector<std_msgs::Float64> lateralCommand;
std::vector<std_msgs::Float64> verticalCommand;
std::vector<std_msgs::Float64> rudderCommand;

void clearVectors()
{
    forwardEnables.clear();
    lateralEnables.clear();
    verticalEnables.clear();
    rudderEnables.clear();

    forwardStates.clear();
    forwardSetpoints.clear();

    lateralStates.clear();
    lateralSetpoints.clear();

    verticalStates.clear();
    verticalSetpoints.clear();

    rudderStates.clear();
    rudderSetpoints.clear();

    forwardCommand.clear();
    lateralCommand.clear();
    verticalCommand.clear();
    rudderCommand.clear();
}

void forwardEnableCB(const std_msgs::BoolPtr& mesg)
{
    forwardEnables.push_back(*mesg);
}

void lateralEnableCB(const std_msgs::BoolPtr& mesg)
{
    lateralEnables.push_back(*mesg);
}

void verticalEnableCB(const std_msgs::BoolPtr& mesg)
{
    verticalEnables.push_back(*mesg);
}

void rudderEnableCB(const std_msgs::BoolPtr& mesg)
{
    rudderEnables.push_back(*mesg);
}

void forwardStateCB(const std_msgs::Float64Ptr& mesg)
{
    forwardStates.push_back(*mesg);
}

void forwardSetpointCB(const std_msgs::Float64Ptr& mesg)
{
    forwardSetpoints.push_back(*mesg);
}

void verticalStateCB(const std_msgs::Float64Ptr& mesg)
{
    verticalStates.push_back(*mesg);
}

void verticalSetpointCB(const std_msgs::Float64Ptr& mesg)
{
    verticalSetpoints.push_back(*mesg);
}

void lateralStateCB(const std_msgs::Float64Ptr& mesg)
{
    lateralStates.push_back(*mesg);
}

void lateralSetpointCB(const std_msgs::Float64Ptr& mesg)
{
    lateralSetpoints.push_back(*mesg);
}

void rudderStateCB(const std_msgs::Float64Ptr& mesg)
{
    rudderStates.push_back(*mesg);
}

void rudderSetpointCB(const std_msgs::Float64Ptr& mesg)
{
    rudderSetpoints.push_back(*mesg);
}

void forwardCommandCB(const std_msgs::Float64Ptr& mesg)
{
    forwardCommand.push_back(*mesg);
}

void lateralCommandCB(const std_msgs::Float64Ptr& mesg)
{
    lateralCommand.push_back(*mesg);
}

void verticalCommandCB(const std_msgs::Float64Ptr& mesg)
{
    verticalCommand.push_back(*mesg);
}

void rudderCommandCB(const std_msgs::Float64Ptr& mesg)
{
    rudderCommand.push_back(*mesg);
}

void runSetTargetVelocityTest()
{   
    clearVectors();
    VehicleInfo info;
    FourDOFPropulsionPIDLogic logic(info);

    VehiclePose zeroPose;

    geometry_msgs::Twist vel1;
    vel1.linear.x = 1;
    vel1.linear.y = 0;
    vel1.linear.z = -1;

    vel1.angular.x = 0;
    vel1.angular.y = 0;
    vel1.angular.z = 0;

    geometry_msgs::Twist vel2;
    vel2.linear.x = 2;
    vel2.linear.y = 0;
    vel2.linear.z = std::numeric_limits<double>::quiet_NaN();

    vel2.angular.x = 0;
    vel2.angular.y = 0;
    vel2.angular.z = 0;

    geometry_msgs::Twist vel3;
    vel3.linear.x = std::numeric_limits<double>::infinity();
    vel3.linear.y = 0;
    vel3.linear.z = std::numeric_limits<double>::infinity();

    vel3.angular.x = 0;
    vel3.angular.y = 0;
    vel3.angular.z = std::numeric_limits<double>::infinity();

    logic.setTargetXY(10000,1000);
    logic.setTargetZ(1000);

    logic.setVelocityXY(1, 0, 0);
    logic.setVelocityZ(-1);
    logic.goToXY(zeroPose);
    logic.goToZ(zeroPose);
    ros::spinOnce();
 
    logic.setVelocityXY(2, 0, 0);
    logic.setVelocityZ(-1);
    logic.goToXY(zeroPose);
    logic.goToZ(zeroPose);
    ros::spinOnce();
 
    logic.setVelocityXY(std::numeric_limits<double>::infinity(), 0, std::numeric_limits<double>::infinity());
    logic.setVelocityZ(std::numeric_limits<double>::infinity());
    ros::Duration(1.0).sleep(); //Allows messages to propogate through the ROS system
    ros::spinOnce();

    ASSERT_EQ(1u, forwardEnables.size());
    EXPECT_TRUE(forwardEnables[0].data);

    ASSERT_EQ(1u, verticalEnables.size());
    EXPECT_TRUE(verticalEnables[0].data);

    ASSERT_EQ(1u, rudderEnables.size());
    EXPECT_TRUE(rudderEnables[0].data);


    ASSERT_EQ(2u, forwardSetpoints.size());
    EXPECT_NEAR(1, forwardSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(2, forwardSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2u, verticalSetpoints.size());
    EXPECT_NEAR(-1, verticalSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(-1, verticalSetpoints[1].data, 0.00000001);
    
}


void runGoToXYTest()
{ 
    clearVectors();
    VehicleInfo info;
    FourDOFPropulsionPIDLogic logic(info);

    underwater_vehicle_msgs::VehicleData data1;
    data1.sonarDepth = 100;
    logic.processNewData(data1);
    VehiclePose zeroPose(Eigen::Vector3d(0, 0, 100));

    geometry_msgs::Twist vel1;
    vel1.linear.x = 1;
    vel1.linear.y = 0;
    vel1.linear.z = 1;

    vel1.angular.x = 0;
    vel1.angular.y = 0;
    vel1.angular.z = 0;
        
    logic.setVelocityXY(1, 0, 0);
    logic.setVelocityZ(1);

    logic.setTargetXY(1000, 1000);
    logic.goToXY(zeroPose);
    ros::spinOnce();

    zeroPose.setLinearVelocity(Eigen::Vector3d(1, 0, 0));
    logic.setTargetXY(1000, -1000);
    logic.goToXY(zeroPose);
    ros::Duration(1.0).sleep(); //Allows messages to propogate through the ROS system
    ros::spinOnce();

    ASSERT_EQ(1, forwardEnables.size());
    EXPECT_TRUE(forwardEnables[0].data);

    ASSERT_EQ(1, rudderEnables.size());
    EXPECT_TRUE(rudderEnables[0].data);

    ASSERT_EQ(2, forwardSetpoints.size());
    EXPECT_NEAR(1, forwardSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(1, forwardSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2, forwardStates.size());
    EXPECT_NEAR(0, forwardStates[0].data, 0.00000001);
    EXPECT_NEAR(1, forwardStates[1].data, 0.00000001);

    ASSERT_EQ(2, rudderSetpoints.size());
    EXPECT_NEAR(0, rudderSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(0, rudderSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2, rudderStates.size());
    EXPECT_NEAR(M_PI / 4, rudderStates[0].data, 0.00000001);
    EXPECT_NEAR(-M_PI / 4, rudderStates[1].data, 0.00000001);

}

void runGoToZTest()
{ 
    clearVectors();
    VehicleInfo info;
    FourDOFPropulsionPIDLogic logic(info);

    underwater_vehicle_msgs::VehicleData data1;
    data1.sonarDepth = 100;
    logic.processNewData(data1);
    VehiclePose zeroPose(Eigen::Vector3d(0, 0, 100));

    geometry_msgs::Twist vel1;
    vel1.linear.x = 1;
    vel1.linear.y = 0;
    vel1.linear.z = 1;
        
    logic.setVelocityXY(1, 0, 0);
    logic.setVelocityZ(1);

    logic.setTargetZ(1000);
    logic.goToZ(zeroPose);
    ros::spinOnce();

    zeroPose.setLinearVelocity(Eigen::Vector3d(0, 0, 1));
    logic.setTargetZ(0);
    logic.goToZ(zeroPose);
    ros::Duration(1.0).sleep();
    ros::spinOnce();

    ASSERT_EQ(1, verticalEnables.size());
    EXPECT_TRUE(verticalEnables[0].data);

    ASSERT_EQ(2, verticalSetpoints.size());
    EXPECT_NEAR(1, verticalSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(-1, verticalSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2, verticalStates.size());
    EXPECT_NEAR(0, verticalStates[0].data, 0.00000001);
    EXPECT_NEAR(1, verticalStates[1].data, 0.00000001);
}

void runFollowHeadingTest()
{ 
    clearVectors();
    VehicleInfo info;
    FourDOFPropulsionPIDLogic logic(info);

    underwater_vehicle_msgs::VehicleData data1;
    data1.sonarDepth = 100;
    logic.processNewData(data1);
    VehiclePose zeroPose(Eigen::Vector3d(0, 0, 100));

    geometry_msgs::Twist vel1;
    vel1.linear.x = 1;
    vel1.linear.y = 0;
    vel1.linear.z = -1;

    vel1.angular.x = 0;
    vel1.angular.y = 0;
    vel1.angular.z = 0;
        
    logic.setVelocityXY(1, 0, 0);
    logic.setVelocityZ(-1);

    logic.setFollowHeading(M_PI / 4);
    logic.followHeading(zeroPose);
    ros::spinOnce();

    zeroPose.setLinearVelocity(Eigen::Vector3d(1, 0, 0));
    logic.setFollowHeading(-M_PI / 4);
    logic.followHeading(zeroPose);
    ros::Duration(1.0).sleep(); //Allows messages to propogate through the ROS system
    ros::spinOnce();

    ASSERT_EQ(1, forwardEnables.size());
    EXPECT_TRUE(forwardEnables[0].data);

    ASSERT_EQ(1, rudderEnables.size());
    EXPECT_TRUE(rudderEnables[0].data);

    ASSERT_EQ(2, forwardSetpoints.size());
    EXPECT_NEAR(1, forwardSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(1, forwardSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2, forwardStates.size());
    EXPECT_NEAR(0, forwardStates[0].data, 0.00000001);
    EXPECT_NEAR(1, forwardStates[1].data, 0.00000001);

    ASSERT_EQ(2, rudderSetpoints.size());
    EXPECT_NEAR(0, rudderSetpoints[0].data, 0.00000001);
    EXPECT_NEAR(0, rudderSetpoints[1].data, 0.00000001);

    ASSERT_EQ(2, rudderStates.size());
    EXPECT_NEAR(M_PI / 4, rudderStates[0].data, 0.00000001);
    EXPECT_NEAR(-M_PI / 4, rudderStates[1].data, 0.00000001);
} 

void runAvoidSeafloorTest()
{   
    clearVectors();
    VehicleInfo info;
    FourDOFPropulsionPIDLogic logic(info);

    VehiclePose zeroPose(Eigen::Vector3d(0, 0, 100));

    geometry_msgs::Twist vel1;
    vel1.linear.x = 1;
    vel1.linear.y = 0;
    vel1.linear.z = 1;

    vel1.angular.x = 0;
    vel1.angular.y = 0;
    vel1.angular.z = 0;
        
    logic.setVelocityXY(1, 0, 0);
    logic.setVelocityZ(1);

    underwater_vehicle_msgs::VehicleData data1;
    data1.sonarDepth = 100;
    logic.processNewData(data1);

    logic.avoidSeafloor(zeroPose);
    ros::spinOnce();

    underwater_vehicle_msgs::VehicleData data2;
    data2.sonarDepth = 1;
    logic.processNewData(data2);

    logic.avoidSeafloor(zeroPose);
    ros::Duration(1.0).sleep(); //Allows messages to propogate through the ROS system
    ros::spinOnce();

    ASSERT_EQ(1, verticalEnables.size());
    EXPECT_TRUE(verticalEnables[0].data);

    ASSERT_EQ(1, verticalSetpoints.size());
    EXPECT_TRUE(verticalSetpoints[0].data < 0);

    ASSERT_EQ(1, verticalStates.size());
    EXPECT_NEAR(0, verticalStates[0].data, 0.0000001);

}

//All tests are combined into one so they do not run in parallel becuase they use the same queue.
TEST(FourDOFPropulsionLogic, all)
{
    runSetTargetVelocityTest();
    runGoToXYTest();
    runGoToZTest();
    runFollowHeadingTest();
    runAvoidSeafloorTest();

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
    ros::init(argc, argv, "four_dof_propulsion_pid_logic_test");

    broadcastStaticTransform();

    ros::NodeHandle nh;

    ros::Subscriber s1 = nh.subscribe("forward_thruster/state", 10, &forwardStateCB);
    ros::Subscriber s2 = nh.subscribe("forward_thruster/setpoint", 10, &forwardSetpointCB);
    ros::Subscriber s3 = nh.subscribe("lateral_thruster/state", 10, &lateralStateCB);
    ros::Subscriber s4 = nh.subscribe("lateral_thruster/setpoint", 10, &lateralSetpointCB);
    ros::Subscriber s5 = nh.subscribe("vertical_thruster/state", 10, &verticalStateCB);
    ros::Subscriber s6 = nh.subscribe("vertical_thruster/setpoint", 10, &verticalSetpointCB);
    ros::Subscriber s7 = nh.subscribe("rudder/state", 10, &rudderStateCB);
    ros::Subscriber s8 = nh.subscribe("rudder/setpoint", 10, &rudderSetpointCB);

    ros::Subscriber s9 = nh.subscribe("command_forward_thruster", 10, &forwardCommandCB);
    ros::Subscriber s10 = nh.subscribe("command_lateral_thruster", 10, &lateralCommandCB);    
    ros::Subscriber s11 = nh.subscribe("command_vertical_thruster", 10, &verticalCommandCB);
    ros::Subscriber s12 = nh.subscribe("command_rudder", 10, &rudderCommandCB);

    ros::Subscriber s13 = nh.subscribe("forward_thruster/pid_enable", 10, &forwardEnableCB);
    ros::Subscriber s14 = nh.subscribe("lateral_thruster/pid_enable", 10, &lateralEnableCB);
    ros::Subscriber s15 = nh.subscribe("vertical_thruster/pid_enable", 10, &verticalEnableCB);
    ros::Subscriber s16 = nh.subscribe("rudder/pid_enable", 10, &rudderEnableCB);

    return RUN_ALL_TESTS();
}
