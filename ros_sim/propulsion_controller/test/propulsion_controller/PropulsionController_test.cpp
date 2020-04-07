#include <gtest/gtest.h>
#include <math.h>
#include <functional>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#include "propulsion_controller/PropulsionController.h"

#include "uth/UthPropulsionLogic.h"

using namespace underwater_autonomy;
/*
TEST(PropulsionController, avoidSeafloor) 
{       
    ros::NodeHandle nh("avoidSeafloor");
    VehicleInfo info;

    ros::ServiceClient goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    goToZClient.waitForExistence();

    underwater_vehicle_msgs::GoToZ goToZMsg;
    goToZMsg.request.depth = 100;
    goToZMsg.request.enable = true;

    rawLogicPtr->setAtZ(false);

    //Test that we call avoid seafloor
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    goToZClient.call(goToZMsg);

    //Test that we do not call avoid seafloor as GoToZ is running
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    spinner.stop();
}

TEST(PropulsionController, goToZNoHoldDepth) 
{
    ros::NodeHandle nh("goToZNoHoldDepth");
    VehicleInfo info;

    ros::ServiceClient goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");

    unsigned int goToZCompleteCalls = 0;
    double lastZCompleteDepth = 0;
    bool lastZCompleteHoldDepth = false;
    auto goToZComplete = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZComplete const >& complete) 
    {
        goToZCompleteCalls++;
        lastZCompleteDepth = complete.getConstMessage().get()->depth;
        lastZCompleteHoldDepth = complete.getConstMessage().get()->holdDepth;
    };

	ros::Subscriber goToZCompleteSub = nh.subscribe<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 10, goToZComplete);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    goToZClient.waitForExistence();

    underwater_vehicle_msgs::GoToZ goToZMsg;
    goToZMsg.request.depth = 100;
    goToZMsg.request.enable = true;
    goToZMsg.request.holdDepth = false;

    rawLogicPtr->setAtZ(false);

    //Before GoToZ
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());


    goToZClient.call(goToZMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During GoToZ
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Complete GoToZ
    rawLogicPtr->setAtZ(true);
    controller.update();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_EQ(1, goToZCompleteCalls);
    EXPECT_DOUBLE_EQ(100, lastZCompleteDepth);
    EXPECT_FALSE(lastZCompleteHoldDepth);

    EXPECT_FALSE(rawLogicPtr->getZMovement());

    spinner.stop();
}


TEST(PropulsionController, goToZCancel) 
{
    ros::NodeHandle nh("goToZCancel");
    VehicleInfo info;

    ros::ServiceClient goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");
    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    goToZClient.waitForExistence();

    underwater_vehicle_msgs::GoToZ goToZMsg;
    goToZMsg.request.depth = 100;
    goToZMsg.request.enable = true;
    goToZMsg.request.holdDepth = false;
    rawLogicPtr->setAtZ(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    goToZClient.call(goToZMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Cancel Goal
    underwater_vehicle_msgs::GoToZ enableMsg;    
    enableMsg.request.enable = false;
    goToZClient.call(enableMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getZMovement());

    //Still do avoid seafloor
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
    spinner.stop();
}


TEST(PropulsionController, goToZHoldDepth) 
{
    ros::NodeHandle nh("goToZHoldDepth");
    VehicleInfo info;

    ros::ServiceClient goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");

    unsigned int goToZCompleteCalls = 0;
    double lastZCompleteDepth = 0;
    bool lastZCompleteHoldDepth = false;
    auto goToZComplete = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToZComplete const >& complete)
    {
        goToZCompleteCalls++;
        lastZCompleteDepth = complete.getConstMessage().get()->depth;
        lastZCompleteHoldDepth = complete.getConstMessage().get()->holdDepth;
    };
	ros::Subscriber goToZCompleteSub = nh.subscribe<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 10, goToZComplete);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();
    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));
    PropulsionController controller(nh, info, std::move(genLogic));
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    goToZClient.waitForExistence();

    underwater_vehicle_msgs::GoToZ goToZMsg;
    goToZMsg.request.depth = 100;
    goToZMsg.request.enable = true;
    goToZMsg.request.holdDepth = true;
    rawLogicPtr->setAtZ(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    ros::spinOnce();

    //Send Goal
    goToZClient.call(goToZMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();


    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_EQ(0, goToZCompleteCalls);
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Check Goal doesn't complete when depth reached
    rawLogicPtr->setAtZ(true);
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(2, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_EQ(0, goToZCompleteCalls);
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    spinner.stop();
}
*/
TEST(PropulsionController, goToXY) 
{
    ros::NodeHandle nh("goToXYComplete");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    VehicleInfo info;
    ros::ServiceClient goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");

    unsigned int goToXYCompleteCalls = 0;
    double lastXYCompleteX = -1;
    double lastXYCompleteY = -1;
    auto goToXYComplete = [&] (const ros::MessageEvent< underwater_vehicle_msgs::GoToXYComplete const >& complete) 
    {
        goToXYCompleteCalls++; 
        lastXYCompleteX = complete.getConstMessage().get()->x;
        lastXYCompleteY = complete.getConstMessage().get()->y;
    };
	ros::Subscriber goToXYCompleteSub = nh.subscribe<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 10, goToXYComplete);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));

    goToXYClient.waitForExistence();
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = 100;
    goToXYMsg.request.y = -100;
    goToXYMsg.request.enable = true;

    goToXYClient.call(goToXYMsg);

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_EQ(0, goToXYCompleteCalls);
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Complete Goal
    rawLogicPtr->setAtXY(true);
    controller.update();
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_EQ(1, goToXYCompleteCalls);
    EXPECT_DOUBLE_EQ(100, lastXYCompleteX);
    EXPECT_DOUBLE_EQ(-100, lastXYCompleteY);
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    spinner.stop();
}

/*
TEST(PropulsionController, goToXYCancel) 
{ 
    ros::NodeHandle nh("goToXYCancel");
    VehicleInfo info;

    ros::ServiceClient goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy", 1000);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    goToXYClient.waitForExistence();

    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = 100;
    goToXYMsg.request.y = 100;
    goToXYMsg.request.enable = true;

    goToXYClient.call(goToXYMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();


    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Cancel Goal
    underwater_vehicle_msgs::GoToXY enableMsg;
    enableMsg.request.enable = false;
    goToXYClient.call(enableMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
    spinner.stop();
}

TEST(PropulsionController, followHeadingCancel) 
{
    ros::NodeHandle nh("followHeadingCancel");
    VehicleInfo info;

    ros::ServiceClient followHeadingClient = nh.serviceClient<underwater_vehicle_msgs::FollowHeading>("follow_heading", 1000);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

	underwater_vehicle_msgs::FollowHeading followHeadingMsg;
    followHeadingMsg.request.heading = 100;
    followHeadingMsg.request.enable = true;

    followHeadingClient.call(followHeadingMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Cancel Goal
    underwater_vehicle_msgs::FollowHeading enableMsg;    
    enableMsg.request.enable = false;
    followHeadingClient.call(enableMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
    spinner.stop();
}

TEST(PropulsionController, xyInterruptHeading) 
{
    ros::NodeHandle nh("xyInterruptHeading");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    VehicleInfo info;

    ros::ServiceClient followHeadingClient = nh.serviceClient<underwater_vehicle_msgs::FollowHeading>("follow_heading", 1000);
    ros::ServiceClient goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy", 1000);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));

    PropulsionController controller(nh, info, std::move(genLogic));
    
    rawLogicPtr->setAtXY(false);


    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());


    ros::spinOnce();

    //Send Goal
    underwater_vehicle_msgs::FollowHeading followHeadingMsg;
    followHeadingMsg.request.heading = 100;
    followHeadingMsg.request.enable = true;

    followHeadingClient.call(followHeadingMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Interrupt Goal
    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = 100;
    goToXYMsg.request.y = 100;
    goToXYMsg.request.enable = true;

    goToXYClient.call(goToXYMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Next Goal
    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    spinner.stop();
}

TEST(PropulsionController, headingInterruptXY) 
{
    ros::NodeHandle nh("headingInterruptXY");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    VehicleInfo info;
       
    ros::ServiceClient followHeadingClient = nh.serviceClient<underwater_vehicle_msgs::FollowHeading>("follow_heading", 1000);
    ros::ServiceClient goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy", 1000);

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = 100;
    goToXYMsg.request.y = 100;
    goToXYMsg.request.enable = true;

    goToXYClient.call(goToXYMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Interrupt Goal
    underwater_vehicle_msgs::FollowHeading followHeadingMsg;
    followHeadingMsg.request.heading = 100;
    followHeadingMsg.request.enable = true;

    followHeadingClient.call(followHeadingMsg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    //During Next Goal
    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    spinner.stop();
}
*/
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
    ros::init(argc, argv, "propulsion_controller_∂test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
