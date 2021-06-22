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
#include "underwater_vehicle_msgs/GoToXY.h"

#include "ros_sim_plan_server/command_executors/WaypointsSimCommandExecutor.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"


using namespace underwater_autonomy;

std::shared_ptr<WaypointsCommand> actionExecutePropModuleTypeFail;
std::shared_ptr<WaypointsCommand> actionExecuteAndPause;
std::shared_ptr<WaypointsCommand> actionExecuteAndSucceed;
std::shared_ptr<WaypointsCommand> actionExecuteAndOutOfRegion;
std::shared_ptr<WaypointsCommand> actionTimeReplan;
std::shared_ptr<WaypointsCommand> actionDistanceReplan;
std::shared_ptr<WaypointsCommand> actionPointReachedReplan;

struct CallbackInfo {
    uint goToXYCalls = 0;
    bool enable;
    double x;
    double y;
    double xLinearVelocity;
    double zAngularVelocity;

    ros::ServiceServer goToXYServer;

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;

    ros::Publisher propStatePub;
};

bool propEnable(underwater_vehicle_msgs::GoToXY::Request  &req, 
               underwater_vehicle_msgs::GoToXY::Response &res,
               uint* goToXYCalls,
               bool* enable,
               double* x,
               double* y,
               double* xLinearVelocity,
               double* zAngularVelocity) 
{
    (*goToXYCalls)++;
    *enable = req.enable;
    *x = req.x;
    *y = req.y;
    *xLinearVelocity = req.xLinearVelocity;
    *zAngularVelocity = req.zAngularVelocity;

    return true;
};

void setupCallbacks(ros::NodeHandle& nh, CallbackInfo& callbackInfo)
{
    boost::function<bool (underwater_vehicle_msgs::GoToXY::Request  &req, 
                          underwater_vehicle_msgs::GoToXY::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &callbackInfo.goToXYCalls, 
                                                                                                                           &callbackInfo.enable,
                                                                                                                           &callbackInfo.x,
                                                                                                                           &callbackInfo.y,
                                                                                                                           &callbackInfo.xLinearVelocity,
                                                                                                                           &callbackInfo.zAngularVelocity));
    callbackInfo.goToXYServer = nh.advertiseService("go_to_xy", propSrvFunction);

    callbackInfo.propStatePub = nh.advertise<underwater_vehicle_msgs::PropulsionControllerState>("prop_state", 2);

    ros::ServiceClient propServiceClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");
    propServiceClient.waitForExistence();
}

void waitForState(Command& action, Command::State state) {
    while(action.getState() != state)
    {
        action.monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
}

TEST(WaypointsSimCommandExecutor, ExecuteAndPause)
{
    ros::NodeHandle nh("ExecuteAndPause");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionExecuteAndPause->execute(ros::Time::now().toSec());

    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndPause->getState());

    actionExecuteAndPause->pause(ros::Time::now().toSec());

    EXPECT_EQ(2u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Command::State::PAUSED, actionExecuteAndPause->getState());
}

TEST(WaypointsSimCommandExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    //Execute Command
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());

    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 100;
    poseMsg.pose.pose.position.y = -100;
    poseMsg.pose.pose.position.z = 0;
    posePub.publish(poseMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    state.xyComplete = true;
    state.xySeqNum = 1;
    state.x = 1;
    state.y = 2;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    EXPECT_EQ(2u, callbackInfo.goToXYCalls);
    EXPECT_EQ(2, callbackInfo.x);
    EXPECT_EQ(3, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);

    //Cancel Command
    actionExecuteAndSucceed->pause(ros::Time::now().toSec());
    EXPECT_EQ(3u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Command::State::PAUSED, actionExecuteAndSucceed->getState());

    //Restart Command
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());
    EXPECT_EQ(4u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(100, callbackInfo.x);
    EXPECT_EQ(-100, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);

    state.xyComplete = true;
    state.xySeqNum = 2;
    state.x = 100;
    state.y = -100;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    EXPECT_EQ(5u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(2, callbackInfo.x);
    EXPECT_EQ(3, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);

    state.xyComplete = true;
    state.xySeqNum = 3;
    state.x = 2;
    state.y = 3;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    EXPECT_EQ(6u, callbackInfo.goToXYCalls);
    EXPECT_EQ(3, callbackInfo.x);
    EXPECT_EQ(4, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionExecuteAndSucceed->getState());

    state.xyComplete = true;
    state.xySeqNum = 4;
    state.x = 3;
    state.y = 4;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    waitForState(*actionExecuteAndSucceed, Command::State::COMPLETED);
    EXPECT_EQ(Command::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(WaypointsSimCommandExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");
    
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
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

TEST(WaypointsSimCommandExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionTimeReplan->execute(0);

    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionTimeReplan->getState());

    state.xyComplete = true;
    state.xySeqNum = 1;
    state.x = 1;
    state.y = 2;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    bool replan = false;
    while(!(replan = actionTimeReplan->triggerReplan())) {
        actionTimeReplan->monitor(3.1);
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(WaypointsSimCommandExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    ros::Duration(1).sleep();
    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionDistanceReplan->execute(0);

    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);

    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Command::State::EXECUTING, actionDistanceReplan->getState());

    state.xyComplete = true;
    state.xySeqNum = 1;
    state.x = 1;
    state.y = 2;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    actionDistanceReplan->monitor(3.1);
    EXPECT_FALSE(actionDistanceReplan->triggerReplan());
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 3.1;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

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

TEST(WaypointsSimCommandExecutor, PointReachedReplan)
{
    ros::NodeHandle nh("PointReachedReplan");

    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    underwater_vehicle_msgs::PropulsionControllerState state;
    state.xyComplete = false;
    state.xySeqNum = 0;
    callbackInfo.propStatePub.publish(state);

    actionPointReachedReplan->execute(ros::Time::now().toSec());

    EXPECT_EQ(1, callbackInfo.xLinearVelocity);
    EXPECT_EQ(2, callbackInfo.zAngularVelocity);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Command::State::EXECUTING, actionPointReachedReplan->getState());

    actionPointReachedReplan->monitor(ros::Time::now().toSec());
    EXPECT_FALSE(actionPointReachedReplan->triggerReplan());

    state.xyComplete = true;
    state.xySeqNum = 1;
    state.x = 1;
    state.y = 2;
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);
    callbackInfo.propStatePub.publish(state);

    bool replan = false;
    while(!replan)
    {
        actionPointReachedReplan->monitor(ros::Time::now().toSec());
        replan = actionPointReachedReplan->triggerReplan();
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionPointReachedReplan->triggerReplan());
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
    ros::init(argc, argv, "point_path_sim_action_executor");

    broadcastStaticTransform();

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecutePropModuleTypeFail("ExecutePropModuleTypeFail");
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    points.push_back(Eigen::Vector3d(3,4,5));

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    WaypointsCommand::ReplanType::NONE,
                                                                                    3));
    actionExecuteAndPause->initCommandExecutor();

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    WaypointsCommand::ReplanType::NONE,
                                                                                    3));
    actionExecuteAndSucceed->initCommandExecutor();

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                                        1,
                                                                                        2,
                                                                                        3,
                                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                        WaypointsCommand::ReplanType::NONE,
                                                                                        3));
    actionExecuteAndOutOfRegion->initCommandExecutor();

    ros::NodeHandle nhTimeReplan("TimeReplan");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                            1,
                                                                            2,
                                                                            3,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            WaypointsCommand::ReplanType::PERIODIC_TIME,
                                                                            3));
    actionTimeReplan->initCommandExecutor();

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                WaypointsCommand::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionDistanceReplan->initCommandExecutor();

    ros::NodeHandle nhPointReachedReplan("PointReachedReplan");
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1,  nhPointReachedReplan, info));
    actionPointReachedReplan = std::shared_ptr<WaypointsCommand>(new WaypointsCommand(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    WaypointsCommand::ReplanType::ON_POINT_REACHED,
                                                                                    3));
    actionPointReachedReplan->initCommandExecutor();


    return RUN_ALL_TESTS();
}
