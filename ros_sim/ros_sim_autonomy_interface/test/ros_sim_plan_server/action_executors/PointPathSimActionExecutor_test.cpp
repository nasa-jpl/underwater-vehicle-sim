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
#include "propulsion_controller/PropulsionControllerEnable.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"


using namespace underwater_autonomy;

std::shared_ptr<PointPathAction> actionExecutePropModuleTypeFail;
std::shared_ptr<PointPathAction> actionExecuteAndCancel;
std::shared_ptr<PointPathAction> actionExecuteAndSucceed;
std::shared_ptr<PointPathAction> actionExecuteAndOutOfRegion;
std::shared_ptr<PointPathAction> actionTimeReplan;
std::shared_ptr<PointPathAction> actionDistanceReplan;
std::shared_ptr<PointPathAction> actionPointReachedReplan;

bool propEnable(propulsion_controller::PropulsionControllerEnable::Request  &req, 
               propulsion_controller::PropulsionControllerEnable::Response &res,
               uint* enable) 
{
    (*enable)++;
    return true;
};


TEST(PointPathSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");

    actionExecutePropModuleTypeFail->execute(ros::Time::now().toSec());
    EXPECT_EQ(Action::State::FAILED, actionExecutePropModuleTypeFail->getState());

}

TEST(PointPathSimActionExecutor, ExecuteAndCancel)
{
    ros::NodeHandle nh("ExecuteAndCancel");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndCancel->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionExecuteAndCancel->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndCancel->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    actionExecuteAndCancel->cancel(ros::Time::now().toSec());

    EXPECT_EQ(1, goToXYEnableCalls);
    EXPECT_EQ(Action::State::INTERRUPTED, actionExecuteAndCancel->getState());
    spinner.stop();
}

TEST(PointPathSimActionExecutor, ExecuteAndSucceed)
{
    ros::NodeHandle nh("ExecuteAndSucceed");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Execute Action
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 100;
    poseMsg.pose.pose.position.y = -100;
    poseMsg.pose.pose.position.z = 0;
    posePub.publish(poseMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    while(goToXYCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, goToXYCalls);
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    //Cancel Action
    actionExecuteAndSucceed->cancel(ros::Time::now().toSec());
    EXPECT_EQ(1, goToXYEnableCalls);
    EXPECT_EQ(Action::State::INTERRUPTED, actionExecuteAndSucceed->getState());

    //Restart Action
    actionExecuteAndSucceed->execute(ros::Time::now().toSec());
    while(goToXYCalls != 3)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(3, goToXYCalls);

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(100, x);
    EXPECT_EQ(-100, y);

    completeMsg.x = 100;
    completeMsg.y = -100;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    while(goToXYCalls != 4)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(4, goToXYCalls);

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    completeMsg.x = 2;
    completeMsg.y = 3;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    while(goToXYCalls != 5)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(5, goToXYCalls);

    while(actionExecuteAndSucceed->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndSucceed->getState());
    EXPECT_EQ(3, x);
    EXPECT_EQ(4, y);

    completeMsg.x = 3;
    completeMsg.y = 4;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    actionExecuteAndSucceed->monitor(ros::Time::now().toSec());

    while(actionExecuteAndSucceed->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, actionExecuteAndSucceed->getState());
    spinner.stop();
}

TEST(PointPathSimActionExecutor, ExecuteAndOutOfRegion)
{
    ros::NodeHandle nh("ExecuteAndOutOfRegion");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionExecuteAndOutOfRegion->execute(ros::Time::now().toSec());
    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionExecuteAndOutOfRegion->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionExecuteAndOutOfRegion->getState());

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

TEST(PointPathSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionTimeReplan->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionTimeReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionTimeReplan->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();
    actionTimeReplan->monitor(ros::Time::now().toSec());
    EXPECT_TRUE(actionTimeReplan->triggerReplan());
    EXPECT_FALSE(actionTimeReplan->triggerReplan());
    spinner.stop();
}


TEST(PointPathSimActionExecutor, DistanceReplan)
{
    ros::NodeHandle nh("DistanceReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    actionDistanceReplan->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionDistanceReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionDistanceReplan->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();

    actionDistanceReplan->monitor(ros::Time::now().toSec());
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
    spinner.stop();
}

TEST(PointPathSimActionExecutor, PointReachedReplan)
{
    ros::NodeHandle nh("PointReachedReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("primary_navigation", 2);

    double x = 0;
    double y = 0;
    unsigned int goToXYCalls = 0;
    auto goToXY = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXY const >& goToXY) 
    {x = goToXY.getConstMessage().get()->x;
     y = goToXY.getConstMessage().get()->y;
     goToXYCalls++;};

	ros::Subscriber goToXYSub = nh.subscribe<underwater_vehicle_msgs::GoToXY>("go_to_xy", 10, goToXY);

    uint goToXYEnableCalls = 0;
    boost::function<bool (propulsion_controller::PropulsionControllerEnable::Request  &req, 
                          propulsion_controller::PropulsionControllerEnable::Response &res)> propSrvFunction(boost::bind(&propEnable, _1, _2, &goToXYEnableCalls));

    ros::ServiceServer propService = nh.advertiseService("go_to_xy_enable", propSrvFunction);

    ros::ServiceClient propServiceClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_xy_enable");
    while(!propServiceClient.exists()) {ros::spinOnce();}

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    actionPointReachedReplan->execute(ros::Time::now().toSec());

    while(latestVelMsg == NULL)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(goToXYCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYCalls);

    while(actionPointReachedReplan->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, actionPointReachedReplan->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    actionPointReachedReplan->monitor(ros::Time::now().toSec());
    EXPECT_FALSE(actionPointReachedReplan->triggerReplan());

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();

    bool replan = false;
    while(!replan)
    {
        actionPointReachedReplan->monitor(ros::Time::now().toSec());
        replan = actionPointReachedReplan->triggerReplan();
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(actionPointReachedReplan->triggerReplan());
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

    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhExecutePropModuleTypeFail, invalidInfo));
    actionExecutePropModuleTypeFail = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                            0,
                                                                                            0,
                                                                                            0,
                                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                            PointPathAction::ReplanType::NONE,
                                                                                            0));

    ros::NodeHandle nhExecuteAndCancel("ExecuteAndCancel");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhExecuteAndCancel, info));
    actionExecuteAndCancel = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::NONE,
                                                                                    3));

    ros::NodeHandle nhExecuteAndSucceed("ExecuteAndSucceed");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhExecuteAndSucceed, info));
    actionExecuteAndSucceed = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::NONE,
                                                                                    3));

    ros::NodeHandle nhExecuteAndOutOfRegion("ExecuteAndOutOfRegion");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhExecuteAndOutOfRegion, info));
    actionExecuteAndOutOfRegion = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                        1,
                                                                                        2,
                                                                                        3,
                                                                                        std::unique_ptr<OperationRegion>(new BoxOperationRegion(0, 0, 0, 100, 100, 100)),
                                                                                        PointPathAction::ReplanType::NONE,
                                                                                        3));

    ros::NodeHandle nhTimeReplan("TimeReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhTimeReplan, info));
    actionTimeReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                            1,
                                                                            2,
                                                                            3,
                                                                            std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                            PointPathAction::ReplanType::PERIODIC_TIME,
                                                                            3));

    ros::NodeHandle nhDistanceReplan("DistanceReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhDistanceReplan, info));
    actionDistanceReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                1,
                                                                                2,
                                                                                3,
                                                                                std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                3));

    ros::NodeHandle nhPointReachedReplan("PointReachedReplan");
    PointPathAction::setExecutorCreateFunction(std::bind(&PointPathSimActionExecutor::create, nhPointReachedReplan, info));
    actionPointReachedReplan = std::shared_ptr<PointPathAction>(new PointPathAction(points,
                                                                                    1,
                                                                                    2,
                                                                                    3,
                                                                                    std::unique_ptr<OperationRegion>(new BoxOperationRegion()),
                                                                                    PointPathAction::ReplanType::ON_POINT_REACHED,
                                                                                    3));


    return RUN_ALL_TESTS();
}
