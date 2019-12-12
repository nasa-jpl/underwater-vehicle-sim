#include <gtest/gtest.h>

#include <iostream>

#include "ros/ros.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/GoToZ.h"

#include "ros_sim_plan_server/action_executors/HoldDepthSimActionExecutor.h"


using namespace underwater_autonomy;

TEST(HoldDepthSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(0,
                                                                0,
                                                                0,
                                                                0,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::NONE,
                                                                0,
                                                                NULL)); 

                                                            
    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "Invalid";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);
    EXPECT_FALSE(executor.execute(action));
}

TEST(HoldDepthSimActionExecutor, ExecuteAndCancel)
{
    ros::NodeHandle nh("ExecuteAndCancel");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::NONE,
                                                                4,
                                                                NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(5, depth);

    executor.cancel(action);
    while(holdDepthEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthEnableCalls);

    while(action->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());
}


TEST(HoldDepthSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::NONE,
                                                                4,
                                                                NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(5, depth);


    ros::Duration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, action->getState());

}

TEST(HoldDepthSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                100,
                                                                2,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::NONE,
                                                                4,
                                                                NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(5, depth);

    ros::Duration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::FAILED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, action->getState());
}

TEST(HoldDepthSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                100,
                                                                100,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::PERIODIC_TIME,
                                                                3,
                                                                NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(5, depth);

    ros::Duration(3).sleep();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));

}

TEST(HoldDepthSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                100,
                                                                100,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                3,
                                                                NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    HoldDepthSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(5, depth);

    executor.monitor(action);
    EXPECT_FALSE(executor.triggerReplan(action));
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 3.1;

    posePub.publish(poseMsg);

    bool replan = false;
    while(!replan)
    {
        executor.monitor(action);
        replan = executor.triggerReplan(action);
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(executor.triggerReplan(action));
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
    ros::init(argc, argv, "hold_depth_sim_action_executor");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
