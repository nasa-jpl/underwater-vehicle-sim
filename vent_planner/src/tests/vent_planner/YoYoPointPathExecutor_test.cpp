#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "planner_framework/Action.h"
#include "planner_framework/Plan.h"

#include "planner_framework/PlanDispatcher.h"

#include "vent_planner/actions/YoYoPointPathAction.h"
#include "vent_planner/YoYoPointPathSimActionExecutor.h"
#include "vent_planner/SimVentActionFactory.h"
#include "vehicle_auto_control/Velocity.h"


TEST(FourDOFPropulsionController, YoYoPointPathController){


    ros::NodeHandle nh;
    SimVentActionFactory factory(nh);

    tf::TransformListener listener;

    std::vector<tf::Vector3> points1;
    std::vector<tf::Vector3> points2;

    std::vector<tf::Vector3> allPoints;

    tf::Vector3 point1(20, -20, -100);
    tf::Vector3 point2(20, 0, -100);

    tf::Vector3 point3(20, -30, -100);
    tf::Vector3 point4(10, 0, -100);

    points1.push_back(point1);
    points1.push_back(point2);

    points2.push_back(point3);
    points2.push_back(point4);

    allPoints.push_back(point1);
    allPoints.push_back(point2);
    allPoints.push_back(point3);
    allPoints.push_back(point4);

    std::shared_ptr<YoYoPointPathAction> pointPathAction1 = factory.createYoYoPointPathAction("v0",
                                                                                              1.0,
                                                                                              0.349066,
                                                                                              0.785398, //45 deg
                                                                                              -95.0,
                                                                                              -105.0,
                                                                                              points1);

    std::shared_ptr<YoYoPointPathAction> pointPathAction2 = factory.createYoYoPointPathAction("v0",
                                                                                              1.0,
                                                                                              0.349066,
                                                                                              0.785398, //45 deg
                                                                                              -95.0,
                                                                                              -105.0,
                                                                                              points2);
    PlanDispatcher planDispatcher;
    std::shared_ptr<Plan> plan = std::shared_ptr<Plan>(new Plan());
    plan->addAction(pointPathAction1);
    plan->addAction(pointPathAction2);
    planDispatcher.setPlan(plan);

    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Publisher targetVelPub;
    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v0/command_target_velocity", 1000);
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 200.0);

    if(targetVelPub.getNumSubscribers() == 0)
    {

        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v0", ros::Time(0), ros::Duration(200.0));

    planDispatcher.runPlan();

    unsigned currentPoint = 0;

    bool goingUp = true;
    unsigned yoyo = 0;
    bool isActive = false;
    ros::Time start = ros::Time::now();
    while(pointPathAction2->getState() != Action::State::COMPLETED && (ros::Time::now() - start) <= ros::Duration(400.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v0",  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            FAIL();
        }

        double xDistance = fabs(transform.getOrigin().getX() - allPoints[currentPoint].getX());
        double yDistance = fabs(transform.getOrigin().getY() - allPoints[currentPoint].getY());
        
        double xyDistance = sqrt(xDistance * xDistance + yDistance * yDistance);

        if(xyDistance <= 1.0)
        {
            currentPoint++;
        }

        double zDistance = 0;
        if(goingUp)
        {
            zDistance = fabs(transform.getOrigin().getZ() - (-95.0));
        }
        else
        {
             zDistance = fabs(transform.getOrigin().getZ() - (-105.0));
        }
        
        if(zDistance <= 0.25)
        {
            goingUp = !goingUp;
            yoyo++;
        }

        planDispatcher.update();
    }

    ASSERT_EQ(Action::State::COMPLETED, pointPathAction1->getState());
    ASSERT_EQ(2, pointPathAction2->getCurrentPoint());
    ASSERT_EQ(allPoints.size(), currentPoint);
    ASSERT_TRUE(yoyo >= 4);
}


TEST(PlanPrempting, YoYoPointPathController){

    
    ros::NodeHandle nh;
    SimVentActionFactory factory(nh);

    tf::TransformListener listener;

    std::vector<tf::Vector3> points0;
    std::vector<tf::Vector3> points1;
    std::vector<tf::Vector3> points2;

    std::vector<tf::Vector3> allPoints;


    tf::Vector3 point0(0, 0, -100);

    tf::Vector3 point1(20, -20, -100);
    tf::Vector3 point2(-100, 0, -100);

    tf::Vector3 point3(20, -30, -100);
    tf::Vector3 point4(10, 0, -100);

    points0.push_back(point0);
    points1.push_back(point1);
    points1.push_back(point2);

    points2.push_back(point3);
    points2.push_back(point4);

    allPoints.push_back(point0);
    allPoints.push_back(point1);
    allPoints.push_back(point3);
    allPoints.push_back(point4);
    allPoints.push_back(point2);

    std::shared_ptr<YoYoPointPathAction> pointPathAction0 = factory.createYoYoPointPathAction("v0",
                                                                                              1.0,
                                                                                              0.349066,
                                                                                              0.785398, //45 deg
                                                                                              -95.0,
                                                                                              -105.0,
                                                                                              points0);

    std::shared_ptr<YoYoPointPathAction> pointPathAction1 = factory.createYoYoPointPathAction("v0",
                                                                                              1.0,
                                                                                              0.349066,
                                                                                              0.785398, //45 deg
                                                                                              -95.0,
                                                                                              -105.0,
                                                                                              points1);

    std::shared_ptr<YoYoPointPathAction> pointPathAction2 = factory.createYoYoPointPathAction("v0",
                                                                                              1.0,
                                                                                              0.349066,
                                                                                              0.785398, //45 deg
                                                                                              -95.0,
                                                                                              -105.0,
                                                                                              points2);

    PlanDispatcher planDispatcher;
    std::shared_ptr<Plan> plan0 = std::shared_ptr<Plan>(new Plan());
    std::shared_ptr<Plan> plan1 = std::shared_ptr<Plan>(new Plan());

    plan0->addAction(pointPathAction0);
    plan0->addAction(pointPathAction1);
    plan1->addAction(pointPathAction2);
    planDispatcher.setPlan(plan0);
    //Wait for ros time to start
    while(ros::Time::now().toSec() == 0.0);

    //Wait for vehicle_auto_control node to start
    ros::Publisher targetVelPub;
    targetVelPub = nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/v0/command_target_velocity", 1000);
    ros::Time startWait = ros::Time::now();
    while((targetVelPub.getNumSubscribers() == 0) &&
          (ros::Time::now() - startWait).toSec() <= 200.0);

    if(targetVelPub.getNumSubscribers() == 0)
    {
        FAIL();
    }

    //Wait for transforms
    listener.waitForTransform("/world", "/v0", ros::Time(0), ros::Duration(200.0));

    planDispatcher.runPlan();

    unsigned currentPoint = 0;

    bool isActive = false;
    ros::Time start = ros::Time::now();
    unsigned int planPhase = 0; //0 = plan0, 1=plan1, 2=plan0 again
    while((pointPathAction0->getState() != Action::State::COMPLETED || 
           pointPathAction1->getState() != Action::State::COMPLETED ||
           pointPathAction2->getState() != Action::State::COMPLETED) && (ros::Time::now() - start) <= ros::Duration(600.0))
    {
       tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/world", "/v0",  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            FAIL();
        }

        double xDistance = fabs(transform.getOrigin().getX() - allPoints[currentPoint].getX());
        double yDistance = fabs(transform.getOrigin().getY() - allPoints[currentPoint].getY());
        
        double xyDistance = sqrt(xDistance * xDistance + yDistance * yDistance);

        if(xyDistance <= 1.0)
        {
            currentPoint++;
        }

        planDispatcher.update();


        if(pointPathAction1->getCurrentPoint() == 1 && planPhase == 0)
        {
            planDispatcher.setPlan(plan1);
            planDispatcher.runPlan();
            planPhase = 1;
        }

        if(pointPathAction2->getState() == Action::State::COMPLETED && planPhase == 1)
        {
            plan0->resetInterrupted();
            planDispatcher.setPlan(plan0);
            planDispatcher.runPlan();
            planPhase = 2;
        }
    }

    ASSERT_EQ(Action::State::COMPLETED, pointPathAction0->getState());
    ASSERT_EQ(Action::State::COMPLETED, pointPathAction1->getState());
    ASSERT_EQ(Action::State::COMPLETED, pointPathAction1->getState());
    ASSERT_EQ(2, pointPathAction2->getCurrentPoint());
    ASSERT_EQ(allPoints.size(), currentPoint);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "yoyo_point_path_executor_test");

  return RUN_ALL_TESTS();
}