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
#include "underwater_vehicle_msgs/FollowHeading.h"

#include "ros_sim_plan_server/action_executors/FollowHeadingSimActionExecutor.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"


using namespace underwater_autonomy;

TEST(FollowHeadingSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        0,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        0,
                                                                        NULL)); 

                                                            
    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "Invalid";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);
    EXPECT_FALSE(executor.execute(action));
}


TEST(FollowHeadingSimActionExecutor, ExecuteAndCancel)
{
    ros::NodeHandle nh("ExecuteAndCancel");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        6,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    //Execute action
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);

    //Cancel action
    executor.cancel(action);
    while(followHeadingEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingEnableCalls);

    while(action->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());
}


TEST(FollowHeadingSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        2,
                                                                        2,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        6,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);

    ros::Duration(2).sleep();
    executor.monitor(action);

    while(action->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, action->getState());

}


TEST(FollowHeadingSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        2,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        6,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);

    ros::Duration(2).sleep();
    executor.monitor(action);
    
    while(action->getState() != Action::State::FAILED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, action->getState());

}

TEST(FollowHeadingSimActionExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        6,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 1000;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);

    
    while(action->getState() != Action::State::FAILED)
    {
        executor.monitor(action);
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, action->getState());

}


TEST(FollowHeadingSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::PERIODIC_TIME,
                                                                        3,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);

    ros::WallDuration(3).sleep();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));
}


TEST(FollowHeadingSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double heading = 0;
    unsigned int followHeadingCalls = 0;
    auto followHeading = [&] (const ros::MessageEvent< underwater_vehicle_msgs::FollowHeading const >& followHeading) 
    {heading = followHeading.getConstMessage().get()->heading;
     followHeadingCalls++;};

	ros::Subscriber followHeadingSub = nh.subscribe<underwater_vehicle_msgs::FollowHeading>("follow_heading", 10, followHeading);

    unsigned int followHeadingEnableCalls = 0;
    auto followHeadingEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {followHeadingEnableCalls++;};
	ros::Subscriber followHeadingEnableSub = nh.subscribe<std_msgs::Bool>("follow_heading_enable", 10, followHeadingEnable);

    std::shared_ptr<FollowHeadingAction> action(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        FollowHeadingAction::ReplanType::PERIODIC_DISTANCE,
                                                                        3,
                                                                        NULL)); 

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, latestVelMsg->linear.x);
    EXPECT_EQ(3, latestVelMsg->angular.z);

    while(followHeadingCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, followHeadingCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, heading);
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 3.1;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

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
    ros::init(argc, argv, "follow_heading_sim_action_executor");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
