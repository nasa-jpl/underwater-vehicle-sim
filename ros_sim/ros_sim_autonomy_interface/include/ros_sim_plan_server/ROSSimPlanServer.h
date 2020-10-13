#ifndef ROS_SIM_PLAN_SERVER_H
#define ROS_SIM_PLAN_SERVER_H

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"

#include "ros/ros.h"

class ROSSimPlanServer
{
public:
    ROSSimPlanServer(std::unique_ptr<underwater_autonomy::Planner> planner,
                     std::shared_ptr<ROSSimVehicleInterface> vehicleInterface,
                     int cancelTimeout);
    ROSSimPlanServer(ROSSimPlanServer&& other);
    ~ROSSimPlanServer() {}

    std::string getPlannerState();

    void update();

private:
    std::unique_ptr<underwater_autonomy::Planner> planner;
    std::shared_ptr<ROSSimVehicleInterface> vehicleInterface;

    underwater_autonomy::PlanDispatcher planDispatcher;

    ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif