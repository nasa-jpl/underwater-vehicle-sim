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

#include "ros_sim_plan_server/command_executors/HoldDepthSimCommandExecutor.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<HoldDepthCommand> actionExecutePropModuleTypeFail;
std::shared_ptr<HoldDepthCommand> actionExecuteAndPause;
std::shared_ptr<HoldDepthCommand> actionExecuteAndSucceed;
std::shared_ptr<HoldDepthCommand> actionTimeReplan;
std::shared_ptr<HoldDepthCommand> actionDistanceReplan;
std::shared_ptr<HoldDepthCommand> actionExecuteAndOutOfRegion;


struct CallbackInfo {
    uint goToZCalls = 0;
    bool enable;
    bool holdDepth;
    double depth;
    double zLinearVelocity;
    ros::ServiceServer goToZServer;

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    ros::Subscriber velSub;
};

bool propEnable(underwater_vehicle_msgs::GoToZ::Request  &req, 
               underwater_vehicle_msgs::GoToZ::Response &res,
               uint* goToZCalls,
               bool* enable,
               bool* holdDepth,
               double* depth,
               double* zLinearVelocity)
{
    (*goToZCalls)++;
    *enable = req.enable;
    *holdDepth = req.holdDepth;
    *depth = req.depth;
    *zLinearVelocity = req.zLinearVelocity;
    return true;
};

void setupCallbacks(ros::NodeHandle& nh, CallbackInfo& callbackInfo)
{
    boost::function<bool (underwater_vehicle_msgs::GoToZ::Request  &req, 
                          underwater_vehicle_msgs::GoToZ::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &callbackInfo.goToZCalls, 
                                                                                                                           &callbackInfo.enable,
                                                                                                                           &callbackInfo.holdDepth,
                                                                                                                           &callbackInfo.depth,
                                                                                                                           &callbackInfo.zLinearVelocity));
    callbackInfo.goToZServer = nh.advertiseService("go_to_z", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");
    propServiceClient.waitForExistence();
}

void waitForState(Command& action, Command::State state) {
    while(action.getState() != state)
    {
        action.monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
}

TEST(HoldDepthSimCommandExecutor, ExecuteAndPause)
{
    ros::NodeHandle nh("ExecuteAndPause");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndPause->execute(ros::Time::now().toSec());

    EXPECT_EQ(1, callbackInfo.zLinearVelocity);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndPause->getState());

    actionExecuteAndPause->pause(ros::Time::now().toSec());
    EXPECT_EQ(2u, callbackInfo.goToZCalls);
    EXPECT_FALSE(callbackInfo.enable);
    EXPECT_EQ(Command::State::PAUSED, actionExecuteAndPause->getState());
}

TEST(HoldDepthSimCommandExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndSucceed->execute(0);

    EXPECT_EQ(1, callbackInfo.zLinearVelocity);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(5, callbackInfo.depth);

    actionExecuteAndSucceed->monitor(2.1);
    while(actionExecuteAndSucceed->getState() != Command::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Command::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(YoYoSimCommandExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    EXPECT_EQ(1, callbackInfo.zLinearVelocity);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 1000;

    posePub.publish(poseMsg);
    actionExecuteAndOutOfRegion->monitor(ros::Time::now().toSec());

    waitForState(*actionExecuteAndOutOfRegion, Command::State::FAILED);
    EXPECT_EQ(Command::State::FAILED, actionExecuteAndOutOfRegion->getState());
}

TEST(HoldDepthSimCommandExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);
    actionTimeReplan->execute(0);

    EXPECT_EQ(1, callbackInfo.zLinearVelocity);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Command::State::EXECUTING, actionTimeReplan->getState());

    actionTimeReplan->monitor(3.1);
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(HoldDepthSimCommandExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionDistanceReplan->execute(ros::Time::now().toSec());

    EXPECT_EQ(1, callbackInfo.zLinearVelocity);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Command::State::EXECUTING, actionTimeReplan->getState());

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

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<HoldDepthCommand>(new HoldDepthCommand(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthCommand::ReplanType::NONE,
                                                                                4));
    actionExecuteAndPause->initCommandExecutor();

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<HoldDepthCommand>(new HoldDepthCommand(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthCommand::ReplanType::NONE,
                                                                                4));
    actionExecuteAndSucceed->initCommandExecutor();

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<HoldDepthCommand>(new HoldDepthCommand(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                HoldDepthCommand::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionExecuteAndOutOfRegion->initCommandExecutor();

    ros::NodeHandle nhTimeReplan("TimeReplan");
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1,  nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<HoldDepthCommand>(new HoldDepthCommand(5,
                                                                            1,
                                                                            100,
                                                                            100,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            HoldDepthCommand::ReplanType::PERIODIC_TIME,
                                                                            3));
    actionTimeReplan->initCommandExecutor();

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<HoldDepthCommand>(new HoldDepthCommand(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthCommand::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionDistanceReplan->initCommandExecutor();

    return RUN_ALL_TESTS();
}
