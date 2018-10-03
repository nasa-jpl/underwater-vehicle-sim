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

double SurfaceGradientVentPlanner::planeGradientHeading(const double a0, const double a1)
{
    if(a0 == 0 && a1 == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    tf::Vector3 normal(-a0, -a1, 0);
    tf::Vector3 north(0,1,0);
    tf::Vector3 cross = north.cross(normal);

    if(cross.getZ() >= 0)
    {
        return -north.angle(normal);
    }
    else
    {
        return north.angle(normal);
    }


    return std::numeric_limits<double>::quiet_NaN();
}

bool SurfaceGradientVentPlanner::fitPlane(std::vector<tf::Vector3>& points, double& a0, double& a1, double& b)
{
    //compute mean
    tf::Vector3 mean(0,0,0);

    for(tf::Vector3& point : points)
    {
        mean += point;
    }
    mean /= points.size();

    double xxSum = 0;
    double xySum = 0;
    double xhSum = 0;
    double yySum = 0;
    double yhSum = 0;

    for(tf::Vector3& point : points)
    {
        tf::Vector3 diff = point - mean;
        xxSum += diff.getX() * diff.getX();
        xySum += diff.getX() * diff.getY();
        xhSum += diff.getX() * diff.getZ();
        yySum += diff.getY() * diff.getY();
        yhSum += diff.getY() * diff.getZ();
    }

    double det = xxSum * yySum - xySum * xySum;
    if(det != 0)
    {
        double barA0 = (yySum * xhSum - xySum * yhSum) / det;
        double barA1 = (xxSum * yhSum - xySum * xhSum) / det;

        a0 = barA0;
        a1 = barA1;
        b = mean.getZ() - barA0 * mean.getX() - barA1 * mean.getY();

        return true;
    }

    //Invalid point inputs
    return false;
}