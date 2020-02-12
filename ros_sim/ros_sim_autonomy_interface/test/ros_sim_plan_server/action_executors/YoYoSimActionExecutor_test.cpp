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

#include "propulsion_controller/PropulsionControllerEnable.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<YoYoAction> actionExecutePropModuleTypeFail;
std::shared_ptr<YoYoAction> actionExecuteAndCancel;
std::shared_ptr<YoYoAction> actionExecuteAndSucceed;
std::shared_ptr<YoYoAction> actionExecuteAndOutOfRegion;
std::shared_ptr<YoYoAction> actionTimeReplan;
std::shared_ptr<YoYoAction> actionDistanceReplan;
std::shared_ptr<YoYoAction> actionTurnReplan;

bool propEnable(propulsion_controller::PropulsionControllerEnable::Request  &req, 
               propulsion_controller::PropulsionControllerEnable::Response &res,
               uint* enable) 
{
    (*enable)++;
    return true;
};

TEST(YoYoSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    actionExecutePropModuleTypeFail->execute(ros::Time::now().toSec());
    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());

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

    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndCancel->execute(ros::Time::now().toSec());

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

    while(actionExecuteAndCancel->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndCancel->getState());
    EXPECT_EQ(1, depth);

    actionExecuteAndCancel->cancel(ros::Time::now().toSec());
    while(goToZEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, goToZEnableCalls);
    EXPECT_EQ(Action::State::INTERRUPTED, actionExecuteAndCancel->getState());
    spinner.stop();
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

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher goToZCompletePub = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 2);

    actionExecuteAndSucceed->execute(ros::Time::now().toSec());

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

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(1, depth);

    //Reset goal called
    underwater_vehicle_msgs::GoToZComplete completeMsg;
    completeMsg.depth = 1;
    completeMsg.holdDepth = false;
    goToZCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();

    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());
    while(goToZCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2u, goToZCalls);
    EXPECT_EQ(2, depth);
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());

    ros::WallDuration(2).sleep();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());
    while(actionExecuteAndSucceed->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
    spinner.stop();
}

TEST(YoYoSimActionExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

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

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(3, latestVelMsg->linear.z);

    while(goToZCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToZCalls);

    while(actionExecuteAndOutOfRegion->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 0;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 1000;

    posePub.publish(poseMsg);
    
    while(actionExecuteAndOutOfRegion->getState() != Action::State::FAILED)
    {
        actionExecuteAndOutOfRegion->monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, actionExecuteAndOutOfRegion->getState());
    spinner.stop();
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

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionTimeReplan->execute(ros::Time::now().toSec());

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

    while(actionTimeReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());
    EXPECT_EQ(1, depth);

    ros::Duration(3).sleep();
    actionTimeReplan->monitor(ros::Time::now().toSec());
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
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

    double depth = 0;
    unsigned int goToZCalls = 0;
    auto goToZ = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& goToXY) 
    {depth = goToXY.getConstMessage().get()->depth;
     goToZCalls++;};

	ros::Subscriber goToZSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, goToZ);

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionDistanceReplan->execute(ros::Time::now().toSec());

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

    while(actionDistanceReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());
    EXPECT_EQ(1, depth);

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
    spinner.stop();
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

    uint goToZEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToZEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_z_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionTurnReplan->execute(ros::Time::now().toSec());

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

    while(actionTurnReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionTurnReplan->getState());
    EXPECT_EQ(1, depth);

    underwater_vehicle_msgs::GoToZComplete completeMsg;
    completeMsg.depth = 1;
    completeMsg.holdDepth = false;
    goToZCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();
    
    actionTurnReplan->monitor(ros::Time::now().toSec());
    while(goToZCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2u, goToZCalls);
    EXPECT_EQ(2, depth);
    EXPECT_EQ(Action::State::EXECUTING, actionTurnReplan->getState());

    EXPECT_TRUE(actionTurnReplan->triggerReplan());
    EXPECT_FALSE(actionTurnReplan->triggerReplan());
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

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecutePropModuleTypeFail("ExecutePropModuleTypeFail");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<YoYoAction>(new YoYoAction(0,
                                                                                0,
                                                                                0,
                                                                                0,
                                                                                0,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                YoYoAction::ReplanType::NONE,
                                                                                0)); 

    ros::NodeHandle nhExecuteAndCancel("ExecuteAndCancel");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhExecuteAndCancel, info));
    actionExecuteAndCancel = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        200,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        4));

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        5,
                                                                        6,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        4));

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                        YoYoAction::ReplanType::NONE,
                                                                        6)); 

    ros::NodeHandle nhTimeReplan("TimeReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                2,
                                                                3,
                                                                100,
                                                                200,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                YoYoAction::ReplanType::PERIODIC_TIME,
                                                                3));

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                    2,
                                                                    3,
                                                                    100,
                                                                    200,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    YoYoAction::ReplanType::PERIODIC_DISTANCE,
                                                                    3));

    ros::NodeHandle nhTurnReplan("TurnReplan");
    YoYoAction::setExecutorCreateFunction(std::bind(&YoYoSimActionExecutor::create, nhTurnReplan, info));
    actionTurnReplan = std::shared_ptr<YoYoAction>(new YoYoAction(1,
                                                                2,
                                                                3,
                                                                100,
                                                                200,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                YoYoAction::ReplanType::ON_YOYO_TURN,
                                                                3));
    return RUN_ALL_TESTS();
}
