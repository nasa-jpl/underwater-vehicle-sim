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

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"


using namespace underwater_autonomy;

std::shared_ptr<PointPathAction> actionExecutePropModuleTypeFail;
std::shared_ptr<PointPathAction> actionExecuteAndPause;
std::shared_ptr<PointPathAction> actionExecuteAndSucceed;
std::shared_ptr<PointPathAction> actionExecuteAndOutOfRegion;
std::shared_ptr<PointPathAction> actionTimeReplan;
std::shared_ptr<PointPathAction> actionDistanceReplan;
std::shared_ptr<PointPathAction> actionPointReachedReplan;

struct CallbackInfo {
    uint goToXYCalls = 0;
    bool enable;
    double x;
    double y;
    ros::ServiceServer goToXYServer;

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    ros::Subscriber velSub;

    ros::Publisher propStatePub;
};

bool propEnable(underwater_vehicle_msgs::GoToXY::Request  &req, 
               underwater_vehicle_msgs::GoToXY::Response &res,
               uint* goToXYCalls,
               bool* enable,
               double* x,
               double* y) 
{
    (*goToXYCalls)++;
    *enable = req.enable;
    *x = req.x;
    *y = req.y;

    return true;
};

void velCallback(geometry_msgs::Twist::ConstPtr val, geometry_msgs::Twist::ConstPtr *latestVelMsg) {
    *latestVelMsg = val;
}

void setupCallbacks(ros::NodeHandle& nh, CallbackInfo& callbackInfo)
{
    boost::function<bool (underwater_vehicle_msgs::GoToXY::Request  &req, 
                          underwater_vehicle_msgs::GoToXY::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &callbackInfo.goToXYCalls, 
                                                                                                                           &callbackInfo.enable,
                                                                                                                           &callbackInfo.x,
                                                                                                                           &callbackInfo.y));
    callbackInfo.goToXYServer = nh.advertiseService("go_to_xy", propSrvFunction);

    callbackInfo.velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, boost::bind(&velCallback, _1, &callbackInfo.latestVelMsg));

    callbackInfo.propStatePub = nh.advertise<underwater_vehicle_msgs::PropulsionControllerState>("prop_state", 2);

    ros::ServiceClient propServiceClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");
    propServiceClient.waitForExistence();
}

void waitForState(Action& action, Action::State state) {
    while(action.getState() != state)
    {
        action.monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
}

TEST(PointPathSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CallbackInfo callbackInfo;
    setupCallbacks(nh, callbackInfo);

    actionExecutePropModuleTypeFail->execute(0);
    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());
}

TEST(PointPathSimActionExecutor, ExecuteAndPause)
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

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndPause->getState());

    actionExecuteAndPause->pause(ros::Time::now().toSec());

    EXPECT_EQ(2u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Action::State::PAUSED, actionExecuteAndPause->getState());
}

TEST(PointPathSimActionExecutor, ExecuteAndSucceed)
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

    //Execute Action
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());

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

    //Cancel Action
    actionExecuteAndSucceed->pause(ros::Time::now().toSec());
    EXPECT_EQ(3u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Action::State::PAUSED, actionExecuteAndSucceed->getState());

    //Restart Action
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());
    EXPECT_EQ(4u, callbackInfo.goToXYCalls);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
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
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
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
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());

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

    waitForState(*actionExecuteAndSucceed, Action::State::COMPLETED);
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
}

TEST(PointPathSimActionExecutor, ExecuteAndOutOfRegion)
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
    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 1000;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);

    waitForState(*actionExecuteAndOutOfRegion, Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, actionExecuteAndOutOfRegion->getState());
}

TEST(PointPathSimActionExecutor, TimeReplan)
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

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);

    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());

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

TEST(PointPathSimActionExecutor, DistanceReplan)
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

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);

    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());

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

TEST(PointPathSimActionExecutor, PointReachedReplan)
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

    while(callbackInfo.latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, callbackInfo.latestVelMsg->linear.x);
    EXPECT_EQ(2, callbackInfo.latestVelMsg->angular.z);
    EXPECT_EQ(1u, callbackInfo.goToXYCalls);
    EXPECT_EQ(1, callbackInfo.x);
    EXPECT_EQ(2, callbackInfo.y);
    EXPECT_TRUE(callbackInfo.enable);

    EXPECT_EQ(Action::State::EXECUTING, actionPointReachedReplan->getState());

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

    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                            0,
                                                                                            0,
                                                                                            0,
                                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                            PointPathAction::ReplanType::NONE,
                                                                                            0));
    actionExecutePropModuleTypeFail->initActionExecutor();

    ros::NodeHandle nhExecuteAndPause("ExecuteAndPause");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndPause, info));
    actionExecuteAndPause = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::NONE,
                                                                                    3));
    actionExecuteAndPause->initActionExecutor();

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::NONE,
                                                                                    3));
    actionExecuteAndSucceed->initActionExecutor();

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                        1,
                                                                                        2,
                                                                                        3,
                                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                        PointPathAction::ReplanType::NONE,
                                                                                        3));
    actionExecuteAndOutOfRegion->initActionExecutor();

    ros::NodeHandle nhTimeReplan("TimeReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                            1,
                                                                            2,
                                                                            3,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            PointPathAction::ReplanType::PERIODIC_TIME,
                                                                            3));
    actionTimeReplan->initActionExecutor();

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    actionDistanceReplan->initActionExecutor();

    ros::NodeHandle nhPointReachedReplan("PointReachedReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, std::placeholders::_1,  nhPointReachedReplan, info));
    actionPointReachedReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::ON_POINT_REACHED,
                                                                                    3));
    actionPointReachedReplan->initActionExecutor();


    return RUN_ALL_TESTS();
}
