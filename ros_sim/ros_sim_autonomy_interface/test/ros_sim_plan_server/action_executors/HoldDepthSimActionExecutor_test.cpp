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
#include "propulsion_controller/PropulsionControllerEnable.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<HoldDepthAction> actionExecutePropModuleTypeFail;
std::shared_ptr<HoldDepthAction> actionExecuteAndPause;
std::shared_ptr<HoldDepthAction> actionExecuteAndSucceed;
std::shared_ptr<HoldDepthAction> actionTimeReplan;
std::shared_ptr<HoldDepthAction> actionDistanceReplan;
std::shared_ptr<HoldDepthAction> actionExecuteAndOutOfRegion;


struct CallbackInfo {
    uint goToZCalls = 0;
    bool enable;
    bool holdDepth;
    double depth;
    ros::ServiceServer goToZServer;

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    ros::Subscriber velSub;
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

TEST(HoldDepthSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(0,
                                                                0,
                                                                0,
                                                                0,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::NONE,
                                                                0)); 

    actionExecutePropModuleTypeFail->execute(ros::Time::now().toSec());

    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());
}

TEST(HoldDepthSimActionExecutor, ExecuteAndPause)
{
    ros::NodeHandle nh("ExecuteAndPause");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndPause->execute(ros::Time::now().toSec());

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.z);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndPause->getState());

    actionExecuteAndPause->pause(ros::Time::now().toSec());
    EXPECT_EQ(2u, callbackInfo.goToZCalls);
    EXPECT_FALSE(callbackInfo.enable);
    EXPECT_EQ(Action::State::PAUSED, actionExecuteAndPause->getState());
}

TEST(HoldDepthSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndSucceed->execute(0);

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.z);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(5, callbackInfo.depth);

    actionExecuteAndSucceed->monitor(2.1);
    while(actionExecuteAndSucceed->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(YoYoSimActionExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.z);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 1000;

    posePub.publish(poseMsg);
    actionExecuteAndOutOfRegion->monitor(ros::Time::now().toSec());

    waitForState(*actionExecuteAndOutOfRegion, Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, actionExecuteAndOutOfRegion->getState());
}

TEST(HoldDepthSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);
    actionTimeReplan->execute(0);

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.z);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());

    actionTimeReplan->monitor(3.1);
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(HoldDepthSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionDistanceReplan->execute(ros::Time::now().toSec());

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.z);
    EXPECT_EQ(1u, callbackInfo.goToZCalls);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(5, callbackInfo.depth);
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());

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

    ros::NodeHandle nhExecutePropModuleTypeFail("ExecutePropModuleTypeFail");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(0,
                                                                                        0,
                                                                                        0,
                                                                                        0,
                                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                        HoldDepthAction::ReplanType::NONE,
                                                                                        0));
    actionExecutePropModuleTypeFail->initActionExecutor();

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::NONE,
                                                                                4));
    actionExecuteAndPause->initActionExecutor();

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::NONE,
                                                                                4));
    actionExecuteAndSucceed->initActionExecutor();

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionExecuteAndOutOfRegion->initActionExecutor();

    ros::NodeHandle nhTimeReplan("TimeReplan");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                            1,
                                                                            100,
                                                                            100,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            HoldDepthAction::ReplanType::PERIODIC_TIME,
                                                                            3));
    actionTimeReplan->initActionExecutor();

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionDistanceReplan->initActionExecutor();

    return RUN_ALL_TESTS();
}
