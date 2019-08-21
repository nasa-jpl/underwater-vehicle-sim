#include <gtest/gtest.h>

#include <iostream>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "vehicle_auto_control/GoToZRosAction.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

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
                                                      0,
                                                      NULL)); 

                                                            
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

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = 0;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) 
    { 
        preemptCalled = true;
        goToZServer.setPreempted();
    };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);

    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);

    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    executor.cancel(action);
    while(!preemptCalled);
    EXPECT_TRUE(preemptCalled);
    while(action->getState() != Action::State::INTERRUPTED);
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());

    spinner.stop();
}

TEST(YoYoSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = 0;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) 
    { 
        preemptCalled = true;
        goToZServer.setPreempted();
    };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      2,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);

    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    ros::Duration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, action->getState());

    spinner.stop();
}


TEST(YoYoSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = -1;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) 
    { 
        preemptCalled = true; 
        goToZServer.setPreempted();
    };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      2,
                                                      2,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    //Reset goal called
    goalCalled = false;
    goToZServer.setSucceeded();

    //Wait for next go to z call
    while(!goalCalled);

    EXPECT_EQ(2, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    ros::Duration(2).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::COMPLETED);
    EXPECT_EQ(Action::State::COMPLETED, action->getState());

    spinner.stop();
}

TEST(YoYoSimActionExecutor, ExecuteAndAbort)
{
    ros::NodeHandle nh("ExecuteAndAbort");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = -1;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) { preemptCalled = true; };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::NONE,
                                                      4,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    goToZServer.setAborted();
    while(action->getState() != Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, action->getState());

    spinner.stop();
}


TEST(YoYoSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = -1;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) { preemptCalled = true; };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::PERIODIC_TIME,
                                                      3,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    ros::Duration(3).sleep();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));

    spinner.stop();
}

TEST(YoYoSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);
	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = -1;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) { preemptCalled = true; };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::PERIODIC_DISTANCE,
                                                      3,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

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
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(executor.triggerReplan(action));

    spinner.stop();
}

TEST(YoYoSimActionExecutor, TurnReplan)
{
    ros::NodeHandle nh("TurnReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToZRosAction> goToZServer(nh, "go_to_z", false);

    bool goalCalled = false;
    double z = -1;
    bool holdDepth = false;
    auto goalYoYoCB = [&] (void) 
    {
        vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();
        z = goToZGoal->z;
        holdDepth = goToZGoal->holdDepth;
        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptYoYoCB = [&] (void) 
    { 
        preemptCalled = true; 
        goToZServer.setPreempted();
    };


	goToZServer.registerGoalCallback(goalYoYoCB);
    goToZServer.registerPreemptCallback(preemptYoYoCB);
	goToZServer.start();

    ros::AsyncSpinner spinner(1);
    std::shared_ptr<YoYoAction> action(new YoYoAction(1,
                                                      2,
                                                      3,
                                                      100,
                                                      200,
                                                      NULL,
                                                      YoYoAction::ReplanType::ON_YOYO_TURN,
                                                      3,
                                                      NULL));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    YoYoSimActionExecutor executor(nh, info);

    spinner.start();
    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_TRUE(std::isnan(latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(latestVelMsg->angular.z));
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    //Reset goal called
    goalCalled = false;
    goToZServer.setSucceeded();

    //Wait for next go to z call
    while(!goalCalled);

    EXPECT_EQ(2, z);
    EXPECT_FALSE(holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));

    spinner.stop();
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
