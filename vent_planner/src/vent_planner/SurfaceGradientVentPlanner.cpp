#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"
#include "std_msgs/String.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"

SurfaceGradientVentPlanner::SurfaceGradientVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    vehicleName(vehicleName),
    goalState("running"),
    goalPub(nh.advertise<std_msgs::String>("planner/goal", 1, true)),
    currentPlannerStage(SearchPhase::CALC_GRADIENT)
{
    nh.getParam("planner/fail_time", failTime);
}

std::shared_ptr<Plan> SurfaceGradientVentPlanner::plan()
{
    ROS_INFO("Plan");
  
    std::shared_ptr<Plan> returnPlan;

    switch(currentPlannerStage)
    {
        case SearchPhase::INITIAL_PLAN:
            //Start with spiral for first contact and initial plume height
            break;
        case SearchPhase::CALC_GRADIENT:
            //Create plan for octagon
            //execute plan
            //when plan is completed replan
            break;

        case SearchPhase::FOLLOW_GRADIENT:
            //Implement follow heading or go to point until condition is met
            break;
    }

    //updates the goal state and publishes it
    updateGoal();
    publishGoal();

    return returnPlan;
}

void SurfaceGradientVentPlanner::publishGoal()
{
    std_msgs::String msg;
    msg.data = goalState;
    goalPub.publish(msg);
}

void SurfaceGradientVentPlanner::updateGoal()
{
    if(goalState == "running")
    {
        if(ros::Time::now() >= ros::Time(failTime))
        {
            goalState = "failed";
        }
    }
}