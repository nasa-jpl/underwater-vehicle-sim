#ifndef ROS_SIM_PLAN_SERVER_H
#define ROS_SIM_PLAN_SERVER_H

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"

#include "ros/ros.h"

class ROSSimPlanServer
{
public:
    ROSSimPlanServer(std::unique_ptr<underwater_autonomy::Planner> planner);
    ROSSimPlanServer(ROSSimPlanServer&& other);
    ~ROSSimPlanServer() {}

    void update();

private:
    underwater_autonomy::PlanDispatcher planDispatcher;
    std::unique_ptr<underwater_autonomy::Planner> planner;

    ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif