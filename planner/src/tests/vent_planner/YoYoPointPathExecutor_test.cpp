#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "planner_framework/Action.h"
#include "vent_planner/actions/YoYoPointPathAction.h"

#include "vent_planner/VentSimActionExecutor.h"

#include "vehicle_auto_control/Velocity.h"


TEST(FourDOFPropulsionController, YoYoPointPathController){

    ros::NodeHandle nh;

    VentSimActionExecutor simExecutor(nh, "v0");
    VentActionExecutor& executor = simExecutor;
    
    

    tf::TransformListener listener;

    std::vector<tf::Vector3> points;


    tf::Vector3 point1(20, -20, -100);
    tf::Vector3 point2(20, 0, -100);
    points.push_back(point1);
    points.push_back(point2);

    YoYoPointPathAction pointPathAction(executor,
                                        1.0,
                                        0.349066,
                                        0.785398, //45 deg
                                        -95.0,
                                        -105.0,
                                        points);

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


    pointPathAction.execute();

    unsigned currentPoint = 0;

    bool goingUp = true;
    unsigned yoyo = 0;
    bool isActive = false;
    ros::Time start = ros::Time::now();
    while(currentPoint < points.size() && (ros::Time::now() - start) <= ros::Duration(400.0))
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

        double xDistance = fabs(transform.getOrigin().getX() - points[currentPoint].getX());
        double yDistance = fabs(transform.getOrigin().getY() - points[currentPoint].getY());
        
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

        pointPathAction.monitor();
        if(pointPathAction.getState() == Action::State::EXECUTING)
        {
            isActive = true;
        }
    }

    ros::Time finishWait = ros::Time::now();
    ros::Rate r(1);

    while(pointPathAction.getState() != Action::State::COMPLETED && (ros::Time::now() - finishWait).toSec() <= 40.0)
    {
        pointPathAction.monitor();
        r.sleep();
    }
    ASSERT_EQ(true, isActive);
    ASSERT_EQ(points.size(), currentPoint);
    ASSERT_TRUE(yoyo >= 4);
    ASSERT_EQ(Action::State::COMPLETED, pointPathAction.getState());
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "four_dof_propulsion_controller_test");

  return RUN_ALL_TESTS();
}
