#ifndef ROS_SIM_PLAN_SERVER_H
#define ROS_SIM_PLAN_SERVER_H

#include "underwater_planner/Planner.h"
#include "underwater_planner/PlanDispatcher.h"

#include "ros/ros.h"

class ROSSimPlanServer
{
public:
    ROSSimPlanServer(std::unique_ptr<Planner> planner);
    ROSSimPlanServer(ROSSimPlanServer&& other);
    ~ROSSimPlanServer() {}

    void update();

private:
    PlanDispatcher planDispatcher;
    std::unique_ptr<Planner> planner;

    ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif