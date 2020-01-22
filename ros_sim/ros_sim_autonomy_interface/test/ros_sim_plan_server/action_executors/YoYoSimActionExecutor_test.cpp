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

#include "ros_sim_plan_server/action_executors/YoYoSimActionExecutor.h"


using namespace underwater_autonomy;

TEST(YoYoSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    std::shared_ptr<YoYoAction> action(new YoYoAction(0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      0)); 

                                                            
    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "Invalid";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);
    EXPECT_FALSE(executor.execute(action));
}
 
TEST(YoYoSimActionExecutor, ExecuteAndCancel)
{
    ros::NodeHandle nh("ExecuteAndCancel");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);

    unsigned int goToZEnableCalls = 0;
    auto goToZEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToZEnableCalls++;};
	ros::Subscriber goToZEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, goToZEnable);

    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

    executor.cancel(action);
    while(goToZEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZEnableCalls);

    while(action->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());

}

TEST(YoYoSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);

    unsigned int goToZEnableCalls = 0;
    auto goToZEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToZEnableCalls++;};
	ros::Subscriber goToZEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, goToZEnable);

    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      2,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

    ros::Duration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::FAILED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, action->getState());
}

TEST(YoYoSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);

    unsigned int goToZEnableCalls = 0;
    auto goToZEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToZEnableCalls++;};
	ros::Subscriber goToZEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, goToZEnable);

    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      5,
                                                      6,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

    //Reset goal called
    underwater_vehicle_msgs::GoToZComplete completeMsg;
    completeMsg.depth = 1;
    completeMsg.holdDepth = false;
    goToZCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();

    executor.monitor(action);
    while(goToZCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2u, goToZCalls);
    EXPECT_EQ(2, depth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    ros::WallDuration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, action->getState());

}

TEST(YoYoSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);

    unsigned int goToZEnableCalls = 0;
    auto goToZEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToZEnableCalls++;};
	ros::Subscriber goToZEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, goToZEnable);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::PERIODIC_TIME,
                                                      3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

    ros::Duration(3).sleep();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));
}

TEST(YoYoSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::PERIODIC_DISTANCE,
                                                      3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

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

TEST(YoYoSimActionExecutor, TurnReplan)
{
    ros::NodeHandle nh("TurnReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);
    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::ON_YOYO_TURN,
                                                      3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);


    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, depth);

    underwater_vehicle_msgs::GoToZComplete completeMsg;
    completeMsg.depth = 1;
    completeMsg.holdDepth = false;
    goToZCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();
    
    executor.monitor(action);
    while(goToZCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2u, goToZCalls);
    EXPECT_EQ(2, depth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    EXPECT_TRUE(executor.triggerReplan(action));
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
    ros::init(argc, argv, "yoyo_sim_action_executor");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
