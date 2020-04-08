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

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<YoYoAction> actionExecutePropModuleTypeFail;
std::shared_ptr<YoYoAction> actionExecuteAndPause;
std::shared_ptr<YoYoAction> actionExecuteAndSucceed;
std::shared_ptr<YoYoAction> actionExecuteAndOutOfRegion;
std::shared_ptr<YoYoAction> actionTimeReplan;
std::shared_ptr<YoYoAction> actionDistanceReplan;
std::shared_ptr<YoYoAction> actionTurnReplan;

struct CallbackInfo {
    uint goToZCalls = 0;
    bool enable;
    bool holdDepth;
    double depth;
    ros::ServiceServer goToZServer;

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    ros::Subscriber velSub;

    ros::Publisher propStatePub;
};

bool propEnable(underwater_vehicle_msgs::GoToZ::Request  &req, 
               underwater_vehicle_msgs::GoToZ::Response &res,
               uint* goToZCalls,
               bool* enable,
               bool* holdDepth,
               double* depth)
{
    (*goToZCalls)++;
    *enable = req.enable;
    *holdDepth = req.holdDepth;
    *depth = req.depth;

    return true;
};

void velCallback(geometry_msgs::Twist::ConstPtr val, geometry_msgs::Twist::ConstPtr *latestVelMsg) {
    *latestVelMsg = val;
}

void setupCallbacks(ros::NodeHandle& nh, CallbackInfo& callbackInfo)
{
    boost::function<bool (underwater_vehicle_msgs::GoToZ::Request  &req, 
                          underwater_vehicle_msgs::GoToZ::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &callbackInfo.goToZCalls, 
                                                                                                                           &callbackInfo.enable,
                                                                                                                           &callbackInfo.holdDepth,
                                                                                                                           &callbackInfo.depth));
    callbackInfo.goToZServer = nh.advertiseService("go_to_z", propSrvFunction);

    callbackInfo.velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, boost::bind(&velCallback, _1, &callbackInfo.latestVelMsg));

    callbackInfo.propStatePub = nh.advertise<underwater_vehicle_msgs::PropulsionControllerState>("prop_state", 2);

    ros::ServiceClient propServiceClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");
    propServiceClient.waitForExistence();
}

void waitForState(Action& action, Action::State state) {
    while(action.getState() != state)
    {
        action.monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
}

TEST(YoYoSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    actionExecutePropModuleTypeFail->execute(0);
    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());
}

TEST(YoYoSimActionExecutor, ExecuteAndPause)
{
    ros::NodeHandle nh("ExecuteAndPause");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionExecuteAndPause->execute(ros::Time::now().toSec());

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }

    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->angular.z));
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);


    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndPause->getState());
    EXPECT_EQ(1, callbackInfo.depth);

    actionExecuteAndPause->pause(ros::Time::now().toSec());

    EXPECT_EQ(2u, callbackInfo.goToZCalls);
    EXPECT_FALSE(callbackInfo.enable);

    EXPECT_EQ(Action::State::PAUSED, actionExecuteAndPause->getState());
}

TEST(YoYoSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionExecuteAndSucceed->execute(0);

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->angular.z));
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);

    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_FALSE(callbackInfo.holdDepth);
    EXPECT_EQ(1, callbackInfo.depth);

    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());


    //Reset goal called
    state.zComplete = true;
    state.zSeqNum = 1;
    state.z = 1;
    state.holdDepth = false;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    while(callbackInfo.goToZCalls != 2) {ros::spinOnce();}

    EXPECT_EQ(2u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(2, callbackInfo.depth);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());

    actionExecuteAndSucceed->monitor(5.1);
    
    waitForState(*actionExecuteAndSucceed, Action::State::COMPLETED);
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(YoYoSimActionExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);

    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 1000;

    posePub.publish(poseMsg);
    
    waitForState(*actionExecuteAndOutOfRegion, Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, actionExecuteAndOutOfRegion->getState());
}

TEST(YoYoSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionTimeReplan->execute(0);

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->angular.z));
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);

    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(1, callbackInfo.depth);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());

    actionTimeReplan->monitor(3.1);
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(YoYoSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionDistanceReplan->execute(ros::Time::now().toSec());

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->angular.z));
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);

    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(1, callbackInfo.depth);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_FALSE(callbackInfo.holdDepth);
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());

    actionDistanceReplan->monitor(ros::Time::now().toSec());
    EXPECT_FALSE(actionDistanceReplan->triggerReplan());
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 3.1;

    posePub.publish(poseMsg);

    bool replan = false;
    while(!replan)
    {
        actionDistanceReplan->monitor(ros::Time::now().toSec());
        replan = actionDistanceReplan->triggerReplan();
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionDistanceReplan->triggerReplan());
}

TEST(YoYoSimActionExecutor, TurnReplan)
{
    ros::NodeHandle nh("TurnReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.zComplete = false;
    state.zSeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionTurnReplan->execute(0);

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->linear.x));
    EXPECT_TRUE(std::isnan(callbackInfo.latestVelMsg->angular.z));
    EXPECT_EQ(3, callbackInfo.latestVelMsg->linear.z);

    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(1, callbackInfo.depth);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_FALSE(callbackInfo.holdDepth);

    EXPECT_EQ(Action::State::EXECUTING, actionTurnReplan->getState());

    state.zComplete = true;
    state.zSeqNum = 1;
    state.z = 1;
    state.holdDepth = false;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    while(callbackInfo.goToZCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2u, callbackInfo.goToZCalls);
    EXPECT_EQ(2, callbackInfo.depth);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Action::State::EXECUTING, actionTurnReplan->getState());

    bool replan = false;
    while(!replan)
    {
        replan = actionTurnReplan->triggerReplan();
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionTurnReplan->triggerReplan());
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

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecutePropModuleTypeFail("ExecutePropModuleTypeFail");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<YoYoAction>(new YoYoAction(0,
                                                                                0,
                                                                                0,
                                                                                0,
                                                                                0,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                YoYoAction::ReplanType::NONE,
                                                                                0)); 
    actionExecutePropModuleTypeFail->initActionExecutor();

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        200,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        4));
    actionExecuteAndPause->initActionExecutor();

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        5,
                                                                        6,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        4));
    actionExecuteAndSucceed->initActionExecutor();

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        6)); 
    actionExecuteAndOutOfRegion->initActionExecutor();

    ros::NodeHandle nhTimeReplan("TimeReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                2,
                                                                3,
                                                                100,
                                                                200,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                YoYoAction::ReplanType::PERIODIC_TIME,
                                                                3));
    actionTimeReplan->initActionExecutor();

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                    2,
                                                                    3,
                                                                    100,
                                                                    200,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    YoYoAction::ReplanType::PERIODIC_DISTANCE,
                                                                    3));
    actionDistanceReplan->initActionExecutor();

    ros::NodeHandle nhTurnReplan("TurnReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, std::placeholders::_1,  nhTurnReplan, info));
    actionTurnReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                2,
                                                                3,
                                                                100,
                                                                200,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                YoYoAction::ReplanType::ON_YOYO_TURN,
                                                                3));
    actionTurnReplan->initActionExecutor();

    return RUN_ALL_TESTS();
}
