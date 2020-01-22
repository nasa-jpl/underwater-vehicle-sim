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


using namespace underwater_autonomy;

TEST(PointPathSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");
    std::vector<Eigen::Vector3d> points;
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                0,
                                                                0,
                                                                0,
                                                                NULL,
                                                                PointPathAction::ReplanType::NONE,
                                                                0));

                                                            
    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "Invalid";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);
    EXPECT_FALSE(executor.execute(action));
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

    unsigned int goToXYEnableCalls = 0;
    auto goToXYEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToXYEnableCalls++;};
	ros::Subscriber goToXYEnableSub = nh.subscribe<std_msgs::Bool>("go_to_xy_enable", 10, goToXYEnable);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                PointPathAction::ReplanType::PERIODIC_TIME,
                                                                3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

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

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    executor.cancel(action);
    while(goToXYEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYEnableCalls);

    while(action->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());
}

TEST(PointPathSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

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

    unsigned int goToXYEnableCalls = 0;
    auto goToXYEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToXYEnableCalls++;};
	ros::Subscriber goToXYEnableSub = nh.subscribe<std_msgs::Bool>("go_to_xy_enable", 10, goToXYEnable);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                1,
                                                                2,
                                                                1,
                                                                NULL,
                                                                PointPathAction::ReplanType::PERIODIC_TIME,
                                                                3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

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

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    ros::Duration(1).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::FAILED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::FAILED, action->getState());
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

    unsigned int goToXYEnableCalls = 0;
    auto goToXYEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToXYEnableCalls++;};
	ros::Subscriber goToXYEnableSub = nh.subscribe<std_msgs::Bool>("go_to_xy_enable", 10, goToXYEnable);

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    points.push_back(Eigen::Vector3d(3,4,5));
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                PointPathAction::ReplanType::NONE,
                                                                3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    //Execute Action
    EXPECT_TRUE(executor.execute(action));

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

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
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
    executor.monitor(action);

    while(goToXYCalls != 2)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(2, goToXYCalls);
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    //Cancel Action
    executor.cancel(action);
    while(goToXYEnableCalls != 1)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(1, goToXYEnableCalls);

    while(action->getState() != Action::State::INTERRUPTED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());


    //Restart Action
    executor.execute(action);
    while(goToXYCalls != 3)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(3, goToXYCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(100, x);
    EXPECT_EQ(-100, y);

    completeMsg.x = 100;
    completeMsg.y = -100;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    executor.monitor(action);

    while(goToXYCalls != 4)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(4, goToXYCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    completeMsg.x = 2;
    completeMsg.y = 3;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    executor.monitor(action);

    while(goToXYCalls != 5)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(5, goToXYCalls);

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(3, x);
    EXPECT_EQ(4, y);

    completeMsg.x = 3;
    completeMsg.y = 4;
    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    executor.monitor(action);

    while(action->getState() != Action::State::COMPLETED)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::COMPLETED, action->getState());
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

    unsigned int goToXYEnableCalls = 0;
    auto goToXYEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToXYEnableCalls++;};
	ros::Subscriber goToXYEnableSub = nh.subscribe<std_msgs::Bool>("go_to_xy_enable", 10, goToXYEnable);

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                PointPathAction::ReplanType::PERIODIC_TIME,
                                                                3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);


    EXPECT_TRUE(executor.execute(action));

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

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));
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

    unsigned int goToXYEnableCalls = 0;
    auto goToXYEnable = [&] (const ros::MessageEvent< std_msgs::Bool const >& enable) {goToXYEnableCalls++;};
	ros::Subscriber goToXYEnableSub = nh.subscribe<std_msgs::Bool>("go_to_xy_enable", 10, goToXYEnable);

    ros::Publisher goToXYCompletePub = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 2);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(points,
                                                                1,
                                                                2,
                                                                3,
                                                                NULL,
                                                                PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    EXPECT_TRUE(executor.execute(action));

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

    while(action->getState() != Action::State::EXECUTING)
    {
        ros::spinOnce();
    }
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);

    underwater_vehicle_msgs::GoToXYComplete completeMsg;
    completeMsg.x = 1;
    completeMsg.y = 2;

    goToXYCompletePub.publish(completeMsg);
    ros::WallDuration(3).sleep();
    ros::spinOnce();

    executor.monitor(action);
    EXPECT_FALSE(executor.triggerReplan(action));
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 3.1;
    poseMsg.pose.pose.position.y = 0;
    poseMsg.pose.pose.position.z = 0;

    posePub.publish(poseMsg);

    bool replan = false;

    while(!replan)
    {
        executor.monitor(action);
        replan = executor.triggerReplan(action);
        ros::spinOnce();
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(executor.triggerReplan(action));
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

    return RUN_ALL_TESTS();
}
