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

#include "ros_sim_plan_server/command_executors/FollowHeadingSimCommandExecutor.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<FollowHeadingCommand> actionExecutePropModuleTypeFail;
std::shared_ptr<FollowHeadingCommand> actionExecuteAndPause;
std::shared_ptr<FollowHeadingCommand> actionExecuteAndSucceed;
std::shared_ptr<FollowHeadingCommand> actionExecuteAndOutOfRegion;
std::shared_ptr<FollowHeadingCommand> actionTimeReplan;
std::shared_ptr<FollowHeadingCommand> actionDistanceReplan;

struct CallbackInfo {
    uint followHeadingCalls = 0;
    bool enable;
    double heading;
    double xLinearVelocity;
    double zAngularVelocity;
    ros::ServiceServer followHeadingServer;
};

bool propEnable(underwater_vehicle_msgs::FollowHeading::Request  &req, 
               underwater_vehicle_msgs::FollowHeading::Response &res,
               uint* followHeadingCalls,
               bool* enable,
               double* heading,
               double* xLinearVelocity,
               double* zAngularVelocity)
{
    (*followHeadingCalls)++;
    *enable = req.enable;
    *heading = req.heading;
    *xLinearVelocity = req.xLinearVelocity;
    *zAngularVelocity = req.zAngularVelocity;

    return true;
};

void setupCallbacks(ros::NodeHandle& nh, CallbackInfo& callbackInfo)
{
    boost::function<bool (underwater_vehicle_msgs::FollowHeading::Request  &req, 
                          underwater_vehicle_msgs::FollowHeading::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &callbackInfo.followHeadingCalls, 
                                                                                                                           &callbackInfo.enable,
                                                                                                                           &callbackInfo.heading,
                                                                                                                           &callbackInfo.xLinearVelocity,
                                                                                                                           &callbackInfo.zAngularVelocity));
    callbackInfo.followHeadingServer = nh.advertiseService("follow_heading", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<underwater_vehicle_msgs::FollowHeading>("follow_heading");
    propServiceClient.waitForExistence();
}
void waitForState(Command& action, Command::State state) {
    while(action.getState() != state)
    {
        action.monitor(3);
        ros::spinOnce();
    }
}

TEST(FollowHeadingSimCommandExecutor, ExecuteAndPause)
{
    ros::NodeHandle nh("ExecuteAndPause");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    //Execute action
    actionExecuteAndPause->execute(0);

    EXPECT_EQ(2, callbackInfo.xLinearVelocity);
    EXPECT_EQ(3, callbackInfo.zAngularVelocity);
    EXPECT_EQ(1u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(1, callbackInfo.heading);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndPause->getState());

    //Cancel action
    actionExecuteAndPause->pause(3);
    EXPECT_EQ(2u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(Command::State::PAUSED, actionExecuteAndPause->getState());
}


TEST(FollowHeadingSimCommandExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    actionExecuteAndSucceed->execute(0);
    EXPECT_EQ(2, callbackInfo.xLinearVelocity);
    EXPECT_EQ(3, callbackInfo.zAngularVelocity);
    EXPECT_EQ(1u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(1, callbackInfo.heading);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());

    actionExecuteAndSucceed->monitor(2);

    EXPECT_EQ(Command::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(FollowHeadingSimCommandExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    actionExecuteAndOutOfRegion->execute(0);

    EXPECT_EQ(2, callbackInfo.xLinearVelocity);
    EXPECT_EQ(3, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(1, callbackInfo.heading);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 1000;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);
    
    waitForState(*actionExecuteAndOutOfRegion, Command::State::FAILED);
    EXPECT_EQ(Command::State::FAILED, actionExecuteAndOutOfRegion->getState());
}

TEST(FollowHeadingSimCommandExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    actionTimeReplan->execute(0);

    EXPECT_EQ(2, callbackInfo.xLinearVelocity);
    EXPECT_EQ(3, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(1, callbackInfo.heading);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionTimeReplan->getState());

    bool replan = false;
    while(!(replan = actionTimeReplan->triggerReplan())) {
        actionTimeReplan->monitor(3.1);
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(FollowHeadingSimCommandExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    actionDistanceReplan->execute(0);

    EXPECT_EQ(2, callbackInfo.xLinearVelocity);
    EXPECT_EQ(3, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.followHeadingCalls);
    EXPECT_EQ(1, callbackInfo.heading);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionDistanceReplan->getState());
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 3.1;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);

    bool replan = false;
    while(!replan)
    {
        actionDistanceReplan->monitor(3.1);
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
    ros::init(argc, argv, "follow_heading_sim_action_executor");

    broadcastStaticTransform();

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<FollowHeadingCommand>(new FollowHeadingCommand(1,
                                                                    2,
                                                                    3,
                                                                    100,
                                                                    100,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    FollowHeadingCommand::ReplanType::NONE,
                                                                    6));
    actionExecuteAndPause->initCommandExecutor(); //Initialize action exeuctor early so we can update the create function

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<FollowHeadingCommand>(new FollowHeadingCommand(1,
                                                                    2,
                                                                    3,
                                                                    2,
                                                                    2,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    FollowHeadingCommand::ReplanType::NONE,
                                                                    6)); 
    actionExecuteAndSucceed->initCommandExecutor(); //Initialize action exeuctor early so we can update the create function

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<FollowHeadingCommand>(new FollowHeadingCommand(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                        FollowHeadingCommand::ReplanType::NONE,
                                                                        6)); 
    actionExecuteAndOutOfRegion->initCommandExecutor(); //Initialize action exeuctor early so we can update the create function

    ros::NodeHandle nhTimeReplan("TimeReplan");
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1, nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<FollowHeadingCommand>(new FollowHeadingCommand(1,
                                                            2,
                                                            3,
                                                            100,
                                                            100,
                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                            FollowHeadingCommand::ReplanType::PERIODIC_TIME,
                                                            3)); 
    actionTimeReplan->initCommandExecutor(); //Initialize action exeuctor early so we can update the create function

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<FollowHeadingCommand>(new FollowHeadingCommand(1,
                                                                2,
                                                                3,
                                                                100,
                                                                100,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                FollowHeadingCommand::ReplanType::PERIODIC_DISTANCE,
                                                                3)); 
    actionDistanceReplan->initCommandExecutor(); //Initialize action exeuctor early so we can update the create function

    return RUN_ALL_TESTS();
}
