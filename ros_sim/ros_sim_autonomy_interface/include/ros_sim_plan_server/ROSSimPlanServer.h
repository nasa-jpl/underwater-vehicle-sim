#ifndef ROS_SIM_PLAN_SERVER_H
#define ROS_SIM_PLAN_SERVER_H

#include "ros_sim_plan_server/ROSSimVehicleInterface.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"

#include "ros/ros.h"

class ROSSimPlanServer
{
public:
    ROSSimPlanServer(std::unique_ptr<underwater_autonomy::Planner> planner,
                     ROSSimVehicleInterface& vehicleInterface);
    ROSSimPlanServer(ROSSimPlanServer&& other);
    ~ROSSimPlanServer() {}

    std::string getPlannerStatus();

    void update();

private:
    std::unique_ptr<underwater_autonomy::Planner> planner;
    ROSSimVehicleInterface& vehicleInterface;

    underwater_autonomy::PlanDispatcher planDispatcher;

    ros::Publisher clockSpeedPub;
    float speedUpFactor;

    std::string plannerStatus;
};

#endif