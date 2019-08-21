#include <gtest/gtest.h>

#include <iostream>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "vehicle_auto_control/GoToXYRosAction.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"


using namespace underwater_autonomy;

TEST(PointPathSimActionExecutor, ExecutePropModuleTypeFail)
{
    ros::NodeHandle nh("ExecutePropModuleTypeFail");
    std::vector<Eigen::Vector3d> points;
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        0,
                                                        0,
                                                        0,
                                                        points,
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

	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) 
    { 
        preemptCalled = true; 
        goToXYServer.setPreempted();
    };


	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        3,
                                                        points,
                                                        PointPathAction::ReplanType::PERIODIC_TIME,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    executor.cancel(action);
    while(!preemptCalled);
    EXPECT_TRUE(preemptCalled);
    while(action->getState() != Action::State::INTERRUPTED);
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());

    spinner.stop();
}

TEST(PointPathSimActionExecutor, ExecuteAndTimeout)
{
    ros::NodeHandle nh("ExecuteAndTimeout");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) 
    { 
        preemptCalled = true; 
        goToXYServer.setPreempted();
    };


	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        1,
                                                        points,
                                                        PointPathAction::ReplanType::PERIODIC_TIME,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    ros::Duration(1).sleep();
    executor.monitor(action);
    while(action->getState() != Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, action->getState());

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

	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) 
    { 
        preemptCalled = true; 
        goToXYServer.setPreempted();
    };


	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    points.push_back(Eigen::Vector3d(3,4,5));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        3,
                                                        points,
                                                        PointPathAction::ReplanType::NONE,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());
    
    nav_msgs::Odometry poseMsg;
    poseMsg.pose.pose.position.x = 100;
    poseMsg.pose.pose.position.y = -100;
    poseMsg.pose.pose.position.z = 0;
    posePub.publish(poseMsg);


    goalCalled = false;
    goToXYServer.setSucceeded();
    while(!goalCalled);
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    executor.cancel(action);
    while(!preemptCalled);
    while(action->getState() != Action::State::INTERRUPTED);
    EXPECT_EQ(Action::State::INTERRUPTED, action->getState());

    goalCalled = false;
    executor.execute(action);
    while(!goalCalled);
    while(action->getState() != Action::State::EXECUTING);
    EXPECT_EQ(100, x);
    EXPECT_EQ(-100, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    goalCalled = false;
    goToXYServer.setSucceeded();
    while(!goalCalled);
    EXPECT_EQ(2, x);
    EXPECT_EQ(3, y);

    goalCalled = false;
    goToXYServer.setSucceeded();
    while(!goalCalled);
    EXPECT_EQ(3, x);
    EXPECT_EQ(4, y);

    goToXYServer.setSucceeded();

    while(action->getState() != Action::State::COMPLETED);
    EXPECT_EQ(Action::State::COMPLETED, action->getState());

    spinner.stop();
}


TEST(PointPathSimActionExecutor, ExecuteAndAbort)
{
    ros::NodeHandle nh("ExecuteAndAbort");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) { preemptCalled = true; };


	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);
    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        3,
                                                        points,
                                                        PointPathAction::ReplanType::PERIODIC_TIME,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    goToXYServer.setAborted();
    while(action->getState() != Action::State::FAILED);
    EXPECT_EQ(Action::State::FAILED, action->getState());

    spinner.stop();
}

TEST(PointPathSimActionExecutor, TimeReplan)
{
    ros::NodeHandle nh("TimeReplan");

    geometry_msgs::Twist::ConstPtr latestVelMsg = NULL;
    auto velCB = [&] (geometry_msgs::Twist::ConstPtr val)
    { latestVelMsg = val; };
    ros::Subscriber velSub = nh.subscribe<geometry_msgs::Twist>("command_target_velocity", 1, velCB);

	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) { preemptCalled = true; };


	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        3,
                                                        points,
                                                        PointPathAction::ReplanType::PERIODIC_TIME,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    goalCalled = false;
    goToXYServer.setSucceeded();
    while(!goalCalled);

    ros::Duration(3).sleep();
    executor.monitor(action);
    EXPECT_TRUE(executor.triggerReplan(action));
    EXPECT_FALSE(executor.triggerReplan(action));

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
	actionlib::SimpleActionServer<vehicle_auto_control::GoToXYRosAction> goToXYServer(nh, "go_to_xy", false);

    bool goalCalled = false;
    double x = 0;
    double y = 0;

    auto goalGoToXYCB = [&] (void) 
    {
        vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
        x = goToXYGoal->x;
        y = goToXYGoal->y;

        goalCalled = true;
    };

    bool preemptCalled = false;
    auto preemptGoToXYCB = [&] (void) { preemptCalled = true; };

	goToXYServer.registerGoalCallback(goalGoToXYCB);
    goToXYServer.registerPreemptCallback(preemptGoToXYCB);
	goToXYServer.start();

    ros::AsyncSpinner spinner(1);

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d(1,2,3));
    points.push_back(Eigen::Vector3d(2,3,4));
    std::shared_ptr<PointPathAction> action(new PointPathAction(NULL,
                                                        NULL,
                                                        1,
                                                        2,
                                                        3,
                                                        points,
                                                        PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                        3));

    underwater_vehicle_msgs::GetVehicleInfo infoMsg;
    infoMsg.response.propModuleType = "FourDOFPropulsion";
    VehicleInfo info(infoMsg);
    PointPathSimActionExecutor executor(nh, info);

    spinner.start();

    EXPECT_TRUE(executor.execute(action));

    while(latestVelMsg == NULL);
    EXPECT_EQ(1, latestVelMsg->linear.x);
    EXPECT_EQ(2, latestVelMsg->angular.z);

    while(!goalCalled);

    while(action->getState() != Action::State::EXECUTING);

    EXPECT_EQ(1, x);
    EXPECT_EQ(2, y);
    EXPECT_EQ(Action::State::EXECUTING, action->getState());

    goalCalled = false;
    goToXYServer.setSucceeded();
    while(!goalCalled);

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
    }
    EXPECT_TRUE(replan);
    EXPECT_FALSE(executor.triggerReplan(action));

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

    return RUN_ALL_TESTS();
}
