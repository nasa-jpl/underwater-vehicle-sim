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
#include "propulsion_controller/PropulsionControllerState.h"
#include "propulsion_controller/PropulsionControllerEnable.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<FollowHeadingAction> actionExecutePropModuleTypeFail;
std::shared_ptr<FollowHeadingAction> actionExecuteAndCancel;
std::shared_ptr<FollowHeadingAction> actionExecuteAndSucceed;
std::shared_ptr<FollowHeadingAction> actionExecuteAndOutOfRegion;
std::shared_ptr<FollowHeadingAction> actionTimeReplan;
std::shared_ptr<FollowHeadingAction> actionDistanceReplan;

bool propEnable(propulsion_controller::PropulsionControllerEnable::Request  &req, 
               propulsion_controller::PropulsionControllerEnable::Response &res,
               uint* enable) 
{
    (*enable)++;
    return true;
};


TEST(FollowHeadingSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    actionExecutePropModuleTypeFail->execute(ros::Time::now().toSec());
    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());
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

    uint followHeadingEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &followHeadingEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("follow_heading_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("follow_heading_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Execute action
    actionExecuteAndCancel->execute(ros::Time::now().toSec());

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

    while(actionExecuteAndCancel->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndCancel->getState());
    EXPECT_EQ(1, heading);

    //Cancel action
    actionExecuteAndCancel->cancel(ros::Time::now().toSec());
    EXPECT_EQ(1, followHeadingEnableCalls);
    EXPECT_EQ(Action::State::INTERRUPTED, actionExecuteAndCancel->getState());
    spinner.stop();
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

    uint followHeadingEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &followHeadingEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("follow_heading_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("follow_heading_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndSucceed->execute(ros::Time::now().toSec());
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

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(1, heading);

    ros::Duration(2).sleep();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    while(actionExecuteAndSucceed->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
    spinner.stop();
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

    uint followHeadingEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &followHeadingEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("follow_heading_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("follow_heading_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
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

    while(actionExecuteAndOutOfRegion->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());
    EXPECT_EQ(1, heading);

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 1000;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);

    
    while(actionExecuteAndOutOfRegion->getState() != Action::State::FAILED)
    {
        actionExecuteAndOutOfRegion->monitor(ros::Time::now().toSec());
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, actionExecuteAndOutOfRegion->getState());
    spinner.stop();
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

    uint followHeadingEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &followHeadingEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("follow_heading_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("follow_heading_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    actionTimeReplan->execute(ros::Time::now().toSec());
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

    while(actionTimeReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());
    EXPECT_EQ(1, heading);

    ros::WallDuration(3).sleep();
    actionTimeReplan->monitor(ros::Time::now().toSec());
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
    spinner.stop();
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

    uint followHeadingEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &followHeadingEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("follow_heading_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("follow_heading_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    FollowHeadingSimActionExecutor executor(nh, info);

    actionDistanceReplan->execute(ros::Time::now().toSec());

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

    while(actionDistanceReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());
    EXPECT_EQ(1, heading);
    
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
    ros::init(argc, argv, "follow_heading_sim_action_executor");

    broadcastStaticTransform();

    underwater_vehicle_msgs::GetVehicleInfo invalidInfoMsg;
    invalidInfoMsg.response.propModuleType = "Invalid";
    VehicleInfo invalidInfo(invalidInfoMsg);

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);

    ros::NodeHandle nhExecutePropModuleTypeFail("ExecutePropModuleTypeFail");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(0,
                                                                            0,
                                                                            0,
                                                                            0,
                                                                            0,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            FollowHeadingAction::ReplanType::NONE,
                                                                            0));

    ros::NodeHandle nhExecuteAndCancel("ExecuteAndCancel");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhExecuteAndCancel, info));
    actionExecuteAndCancel = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(1,
                                                                    2,
                                                                    3,
                                                                    100,
                                                                    100,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    FollowHeadingAction::ReplanType::NONE,
                                                                    6));

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(1,
                                                                    2,
                                                                    3,
                                                                    2,
                                                                    2,
                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                    FollowHeadingAction::ReplanType::NONE,
                                                                    6)); 

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(1,
                                                                        2,
                                                                        3,
                                                                        100,
                                                                        100,
                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                        FollowHeadingAction::ReplanType::NONE,
                                                                        6)); 

    ros::NodeHandle nhTimeReplan("TimeReplan");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(1,
                                                            2,
                                                            3,
                                                            100,
                                                            100,
                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                            FollowHeadingAction::ReplanType::PERIODIC_TIME,
                                                            3)); 

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    FollowHeadingAction::setExecutorCreateFunction(std::bind(&FollowHeadingSimActionExecutor::create, nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<FollowHeadingAction>(new FollowHeadingAction(1,
                                                                2,
                                                                3,
                                                                100,
                                                                100,
                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                FollowHeadingAction::ReplanType::PERIODIC_DISTANCE,
                                                                3)); 

    return RUN_ALL_TESTS();
}
