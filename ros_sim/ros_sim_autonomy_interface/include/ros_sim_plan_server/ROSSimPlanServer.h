#ifndef ROS_SIM_PLAN_SERVER_H
#define ROS_SIM_PLAN_SERVER_H

#include "underwater_planner/Planner.h"
#include "underwater_planner/PlanDispatcher.h"

#include "ros/ros.h"

class ROSSimPlanServer
{
public:
    ROSSimPlanServer(ros::NodeHandle nh, std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner);
    ROSSimPlanServer(ROSSimPlanServer&& other);
    ~ROSSimPlanServer() {}

    void update();

private:
    std::unique_ptr<PlanDispatcher> planDispatcher;
    std::unique_ptr<Planner> planner;
    ros::NodeHandle nh;

    const ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif