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
#include "propulsion_controller/PropulsionControllerState.h"

#include "underwater_autonomy/util/BoxOperationRegion.h"

using namespace underwater_autonomy;

std::shared_ptr<HoldDepthAction> actionExecutePropModuleTypeFail;
std::shared_ptr<HoldDepthAction> actionExecuteAndCancel;
std::shared_ptr<HoldDepthAction> actionExecuteAndSucceed;
std::shared_ptr<HoldDepthAction> actionTimeReplan;
std::shared_ptr<HoldDepthAction> actionDistanceReplan;
std::shared_ptr<HoldDepthAction> actionExecuteAndOutOfRegion;

bool zState = true;
bool propState(propulsion_controller::PropulsionControllerState::Request  &req, 
               propulsion_controller::PropulsionControllerState::Response &res) 
{
    res.zEnabled = zState; 
    return true;
};

TEST(HoldDepthSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

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

TEST(HoldDepthSimActionExecutor, ExecuteAndCancel)
{
    ros::NodeHandle nh("ExecuteAndCancel");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    ros::ServiceServer stateService = nh.advertiseService("get_propulsion_state", propState);
    ros::ServiceClient propStateClient = nh.serviceClient<propulsion_controller::PropulsionControllerState>("get_propulsion_state");
    while(!propStateClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndCancel->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(actionExecuteAndCancel->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndCancel->getState());
    EXPECT_EQ(5, depth);

    actionExecuteAndCancel->cancel(ros::Time::now().toSec());
    while(holdDepthEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthEnableCalls);

    EXPECT_EQ(Action::State::INTERRUPTING, actionExecuteAndCancel->getState());
    actionExecuteAndCancel->monitor(ros::Time::now().toSec());
    EXPECT_EQ(Action::State::INTERRUPTING, actionExecuteAndCancel->getState());

    zState = false;
    actionExecuteAndCancel->monitor(ros::Time::now().toSec());

    while(actionExecuteAndCancel->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, actionExecuteAndCancel->getState());
    spinner.stop();
}

TEST(HoldDepthSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    actionExecuteAndSucceed->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(5, depth);


    ros::Duration(2).sleep();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());
    while(actionExecuteAndSucceed->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());

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

    unsigned int goToZEnableCalls = 0;
    auto goToZEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToZEnableCalls++;};
	ros::Subscriber goToZEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, goToZEnable);

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

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
}

TEST(HoldDepthSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    actionTimeReplan->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(actionTimeReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());
    EXPECT_EQ(5, depth);

    ros::Duration(3).sleep();
    actionTimeReplan->monitor(ros::Time::now().toSec());
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
}

TEST(HoldDepthSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double depth = 0;
    unsigned int holdDepthCalls = 0;
    auto holdDepth = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZ const >& holdDepth) 
    {depth = holdDepth.getConstMessage().get()->depth;
     holdDepthCalls++;};

	ros::Subscriber holdDepthSub = nh.subscribe<underwater_vehicle_msgs::GoToZ>("go_to_z", 10, holdDepth);

    unsigned int holdDepthEnableCalls = 0;
    auto holdDepthEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {holdDepthEnableCalls++;};
	ros::Subscriber holdDepthEnableSub = nh.subscribe<std_msgs::Bool>("go_to_z_enable", 10, holdDepthEnable);

    std::shared_ptr<HoldDepthAction> action(new HoldDepthAction(5,
                                                                1,
                                                                100,
                                                                100,
                                                                NULL,
                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                3));


    actionDistanceReplan->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.z);

    while(holdDepthCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1u, holdDepthCalls);

    while(actionDistanceReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());
    EXPECT_EQ(5, depth);

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
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(0,
                                                                                        0,
                                                                                        0,
                                                                                        0,
                                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                        HoldDepthAction::ReplanType::NONE,
                                                                                        0));

    ros::NodeHandle nhExecuteAndCancel("ExecuteAndCancel");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhExecuteAndCancel, info));
    actionExecuteAndCancel = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::NONE,
                                                                                4));

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::NONE,
                                                                                4));

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));
    ros::NodeHandle nhTimeReplan("TimeReplan");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                            1,
                                                                            100,
                                                                            100,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            HoldDepthAction::ReplanType::PERIODIC_TIME,
                                                                            3));

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    HoldDepthAction::setExecutorCreateFunction(std::bind(&HoldDepthSimActionExecutor::create, nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<HoldDepthAction>(new HoldDepthAction(5,
                                                                                1,
                                                                                100,
                                                                                100,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                HoldDepthAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));

    return RUN_ALL_TESTS();
}
