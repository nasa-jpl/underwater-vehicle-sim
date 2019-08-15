#include <gtest/gtest.h>
#include <math.h>
#include <functional>

#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#include "actionlib/client/simple_action_client.h"

#include "vehicle_auto_control/PropulsionController.h"

#include "uth/UthPropulsionLogic.h"

using namespace underwater_autonomy;

TEST(PropulsionController, avoidSeafloor) 
{       
    bool goToZDoneCalled = false;
    bool goToZActiveCalled = false;
    auto goToZDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToZRosResultConstPtr& result)
    { goToZDoneCalled = true; };

    auto goToZActive = [&] () { goToZActiveCalled = true; };
    auto goToZFeedback = [&] (const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("avoidSeafloor");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient(nh, "go_to_z", true);
    vehicle_auto_control::GoToZRosGoal goToZGoal;
    ros::spinOnce();
    goToZGoal.timeout = -1;
    goToZGoal.z = 100;

    rawLogicPtr->setAtZ(false);

    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    ros::spinOnce();
    while(!goToZClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToZClient.sendGoal(goToZGoal,
                        goToZDone,
                        goToZActive,
                        goToZFeedback);
    while(!goToZActiveCalled)
    {
        ros::spinOnce();
    }

    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    goToZClient.cancelGoal();
    while(!goToZDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToZNoHoldDepth) 
{
    bool goToZDoneCalled = false;
    bool goToZActiveCalled = false;
    auto goToZDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToZRosResultConstPtr& result)
    { goToZDoneCalled = true; };

    auto goToZActive = [&] () { goToZActiveCalled = true; };
    auto goToZFeedback = [&] (const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToZNoHoldDepth");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient(nh, "go_to_z", true);
    vehicle_auto_control::GoToZRosGoal goToZGoal;
    goToZGoal.timeout = -1;
    goToZGoal.z = 100;
    goToZGoal.holdDepth = false;
    rawLogicPtr->setAtZ(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    ros::spinOnce();

    //Send Goal
    while(!goToZClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToZClient.sendGoal(goToZGoal,
                        goToZDone,
                        goToZActive,
                        goToZFeedback);
    while(!goToZActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Complete Goal
    rawLogicPtr->setAtZ(true);
    controller.update();
    while(!goToZDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToZCancel) 
{
    bool goToZDoneCalled = false;
    bool goToZActiveCalled = false;
    auto goToZDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToZRosResultConstPtr& result)
    { goToZDoneCalled = true; };

    auto goToZActive = [&] () { goToZActiveCalled = true; };
    auto goToZFeedback = [&] (const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToZCancel");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient(nh, "go_to_z", true);
    vehicle_auto_control::GoToZRosGoal goToZGoal;
    goToZGoal.timeout = -1;
    goToZGoal.z = 100;
    goToZGoal.holdDepth = false;
    rawLogicPtr->setAtZ(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    ros::spinOnce();

    //Send Goal
    while(!goToZClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToZClient.sendGoal(goToZGoal,
                        goToZDone,
                        goToZActive,
                        goToZFeedback);
    while(!goToZActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Cancel Goal
    goToZClient.cancelGoal();
    while(!goToZDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToZTimeout) 
{ 
    bool goToZDoneCalled = false;
    bool goToZActiveCalled = false;
    auto goToZDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToZRosResultConstPtr& result)
    { goToZDoneCalled = true; };

    auto goToZActive = [&] () { goToZActiveCalled = true; };
    auto goToZFeedback = [&] (const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToZTimeout");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient(nh, "go_to_z", true);
    vehicle_auto_control::GoToZRosGoal goToZGoal;
    goToZGoal.timeout = 2;
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    while(!goToZClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToZClient.sendGoal(goToZGoal,
                        goToZDone,
                        goToZActive,
                        goToZFeedback);

    while(!goToZActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Timeout Goal

    ros::Duration(2).sleep();

    controller.update();
    while(!goToZDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_FALSE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopZCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToZHoldDepth) 
{
    
    bool goToZDoneCalled = false;
    bool goToZActiveCalled = false;
    auto goToZDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToZRosResultConstPtr& result)
    { goToZDoneCalled = true; };

    auto goToZActive = [&] () { goToZActiveCalled = true; };
    auto goToZFeedback = [&] (const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToZHoldDepth");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient(nh, "go_to_z", true);
    vehicle_auto_control::GoToZRosGoal goToZGoal;
    goToZGoal.timeout = -1;
    goToZGoal.z = 100;
    goToZGoal.holdDepth = true;
    rawLogicPtr->setAtZ(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    ros::spinOnce();

    //Send Goal
    while(!goToZClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToZClient.sendGoal(goToZGoal,
                        goToZDone,
                        goToZActive,
                        goToZFeedback);
    while(!goToZActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Check Goal doesn't complete when depth reached
    rawLogicPtr->setAtZ(true);
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(2, rawLogicPtr->getGoToZCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopZCalls());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
    EXPECT_FALSE(goToZDoneCalled);
}

TEST(PropulsionController, goToXY) 
{
    
    bool goToXYDoneCalled = false;
    bool goToXYActiveCalled = false;
    auto goToXYDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToXYRosResultConstPtr& result)
    { goToXYDoneCalled = true; };

    auto goToXYActive = [&] () { goToXYActiveCalled = true; };
    auto goToXYFeedback = [&] (const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToXYComplete");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient(nh, "go_to_xy", true);
    vehicle_auto_control::GoToXYRosGoal goToXYGoal;
    goToXYGoal.timeout = -1;
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());


    ros::spinOnce();

    //Send Goal
    while(!goToXYClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToXYClient.sendGoal(goToXYGoal,
                        goToXYDone,
                        goToXYActive,
                        goToXYFeedback);
    while(!goToXYActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Complete Goal
    rawLogicPtr->setAtXY(true);
    controller.update();
    while(!goToXYDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToXYCancel) 
{ 
    bool goToXYDoneCalled = false;
    bool goToXYActiveCalled = false;
    auto goToXYDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToXYRosResultConstPtr& result)
    { goToXYDoneCalled = true; };

    auto goToXYActive = [&] () { goToXYActiveCalled = true; };
    auto goToXYFeedback = [&] (const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToXYCancel");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient(nh, "go_to_xy", true);
    vehicle_auto_control::GoToXYRosGoal goToXYGoal;
    goToXYGoal.timeout = -1;
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());


    ros::spinOnce();

    //Send Goal
    while(!goToXYClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToXYClient.sendGoal(goToXYGoal,
                        goToXYDone,
                        goToXYActive,
                        goToXYFeedback);
    while(!goToXYActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());


    //Cancel Goal
    goToXYClient.cancelGoal();
    while(!goToXYDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, goToXYTimeout) 
{ 
    bool goToXYDoneCalled = false;
    bool goToXYActiveCalled = false;
    auto goToXYDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToXYRosResultConstPtr& result)
    { goToXYDoneCalled = true; };

    auto goToXYActive = [&] () { goToXYActiveCalled = true; };
    auto goToXYFeedback = [&] (const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("goToXYTimeout");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient(nh, "go_to_xy", true);
    vehicle_auto_control::GoToXYRosGoal goToXYGoal;
    goToXYGoal.timeout = 2;
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    while(!goToXYClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToXYClient.sendGoal(goToXYGoal,
                        goToXYDone,
                        goToXYActive,
                        goToXYFeedback);

    while(!goToXYActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Timeout Goal

    ros::Duration(2).sleep();

    controller.update();
    while(!goToXYDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(4, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}


TEST(PropulsionController, followHeadingCancel) 
{
    bool followHeadingDoneCalled = false;
    bool followHeadingActiveCalled = false;
    auto followHeadingDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::FollowHeadingRosResultConstPtr& result)
    { followHeadingDoneCalled = true; };

    auto followHeadingActive = [&] () { followHeadingActiveCalled = true; };
    auto followHeadingFeedback = [&] (const vehicle_auto_control::FollowHeadingRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("followHeadingCancel");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::FollowHeadingRosAction> followHeadingClient(nh, "follow_heading", true);
    vehicle_auto_control::FollowHeadingRosGoal followHeadingGoal;
    followHeadingGoal.timeout = -1;
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
    while(!followHeadingClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	followHeadingClient.sendGoal(followHeadingGoal,
                        followHeadingDone,
                        followHeadingActive,
                        followHeadingFeedback);
    while(!followHeadingActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Cancel Goal
    followHeadingClient.cancelGoal();
    while(!followHeadingDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, followHeadingTimeout) 
{ 
    bool followHeadingDoneCalled = false;
    bool followHeadingActiveCalled = false;
    auto followHeadingDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::FollowHeadingRosResultConstPtr& result)
    { followHeadingDoneCalled = true; };

    auto followHeadingActive = [&] () { followHeadingActiveCalled = true; };
    auto followHeadingFeedback = [&] (const vehicle_auto_control::FollowHeadingRosFeedbackConstPtr& feedback) {};

    ros::NodeHandle nh("followHeadingTimeout");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::FollowHeadingRosAction> followHeadingClient(nh, "follow_heading", true);
    vehicle_auto_control::FollowHeadingRosGoal followHeadingGoal;
    followHeadingGoal.timeout = 2;
    rawLogicPtr->setAtXY(false);

    //Before Goal
    controller.update();

    EXPECT_EQ(1, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Send Goal
    while(!followHeadingClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	followHeadingClient.sendGoal(followHeadingGoal,
                        followHeadingDone,
                        followHeadingActive,
                        followHeadingFeedback);

    while(!followHeadingActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Timeout Goal

    ros::Duration(2).sleep();

    controller.update();
    while(!followHeadingDoneCalled)
    {
        ros::spinOnce();
    }

    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    controller.update();
    EXPECT_EQ(4, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_FALSE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, xyInterruptHeading) 
{
    bool followHeadingDoneCalled = false;
    bool followHeadingActiveCalled = false;
    auto followHeadingDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::FollowHeadingRosResultConstPtr& result)
    { followHeadingDoneCalled = true; };

    auto followHeadingActive = [&] () { followHeadingActiveCalled = true; };
    auto followHeadingFeedback = [&] (const vehicle_auto_control::FollowHeadingRosFeedbackConstPtr& feedback) {};


    bool goToXYDoneCalled = false;
    bool goToXYActiveCalled = false;
    auto goToXYDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToXYRosResultConstPtr& result)
    { goToXYDoneCalled = true; };

    auto goToXYActive = [&] () { goToXYActiveCalled = true; };
    auto goToXYFeedback = [&] (const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) {};


    ros::NodeHandle nh("xyInterruptHeading");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::FollowHeadingRosAction> followHeadingClient(nh, "follow_heading", true);
    vehicle_auto_control::FollowHeadingRosGoal followHeadingGoal;
    followHeadingGoal.timeout = -1;

    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient(nh, "go_to_xy", true);
    vehicle_auto_control::GoToXYRosGoal goToXYGoal;
    goToXYGoal.timeout = -1;
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
    while(!followHeadingClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	followHeadingClient.sendGoal(followHeadingGoal,
                        followHeadingDone,
                        followHeadingActive,
                        followHeadingFeedback);
    while(!followHeadingActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(0, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Interrupt Goal
    while(!goToXYClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToXYClient.sendGoal(goToXYGoal,
                        goToXYDone,
                        goToXYActive,
                        goToXYFeedback);
    while(!goToXYActiveCalled)
    {
        ros::spinOnce();
    }

    //During Next Goal
    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
}

TEST(PropulsionController, headingInterruptXY) 
{
    bool followHeadingDoneCalled = false;
    bool followHeadingActiveCalled = false;
    auto followHeadingDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::FollowHeadingRosResultConstPtr& result)
    { followHeadingDoneCalled = true; };

    auto followHeadingActive = [&] () { followHeadingActiveCalled = true; };
    auto followHeadingFeedback = [&] (const vehicle_auto_control::FollowHeadingRosFeedbackConstPtr& feedback) {};


    bool goToXYDoneCalled = false;
    bool goToXYActiveCalled = false;
    auto goToXYDone = [&] (const actionlib::SimpleClientGoalState& state,
                         const vehicle_auto_control::GoToXYRosResultConstPtr& result)
    { goToXYDoneCalled = true; };

    auto goToXYActive = [&] () { goToXYActiveCalled = true; };
    auto goToXYFeedback = [&] (const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) {};


    ros::NodeHandle nh("headingInterruptXY");
    VehicleInfo info;

    std::unique_ptr<UthPropulsionLogic> uthLogic(new UthPropulsionLogic(info));
    UthPropulsionLogic* rawLogicPtr = uthLogic.get();

    std::unique_ptr<PropulsionLogicInterface> genLogic(std::move(uthLogic));


    PropulsionController controller(nh, info, std::move(genLogic));
    
    actionlib::SimpleActionClient<vehicle_auto_control::FollowHeadingRosAction> followHeadingClient(nh, "follow_heading", true);
    vehicle_auto_control::FollowHeadingRosGoal followHeadingGoal;
    followHeadingGoal.timeout = -1;

    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient(nh, "go_to_xy", true);
    vehicle_auto_control::GoToXYRosGoal goToXYGoal;
    goToXYGoal.timeout = -1;
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
    while(!goToXYClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	goToXYClient.sendGoal(goToXYGoal,
                        goToXYDone,
                        goToXYActive,
                        goToXYFeedback);
    while(!goToXYActiveCalled)
    {
        ros::spinOnce();
    }

    //During Goal
    controller.update();
    EXPECT_EQ(2, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(0, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(1, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());

    //Interrupt Goal
    while(!followHeadingClient.waitForServer(ros::Duration(1)))
    {
        ros::spinOnce();
    }
	followHeadingClient.sendGoal(followHeadingGoal,
                        followHeadingDone,
                        followHeadingActive,
                        followHeadingFeedback);
    while(!followHeadingActiveCalled)
    {
        ros::spinOnce();
    }

    //During Next Goal
    controller.update();
    EXPECT_EQ(3, rawLogicPtr->getAvoidSeafloorCalls());
    EXPECT_EQ(1, rawLogicPtr->getFollowHeadingCalls());
    EXPECT_EQ(1, rawLogicPtr->getGoToXYCalls());
    EXPECT_EQ(2, rawLogicPtr->getStopXYCalls());
    EXPECT_TRUE(rawLogicPtr->getXYMovement());
    EXPECT_TRUE(rawLogicPtr->getZMovement());
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
    ros::init(argc, argv, "propulsion_controller_∂test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
