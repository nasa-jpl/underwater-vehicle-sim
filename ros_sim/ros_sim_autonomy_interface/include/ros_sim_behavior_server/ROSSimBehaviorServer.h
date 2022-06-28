#ifndef ROS_SIM_BEHAVIOR_SERVER_H
#define ROS_SIM_BEHAVIOR_SERVER_H

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include "underwater_autonomy/behaviors/Behavior.h"
#include "underwater_autonomy/behaviors/BehaviorController.h"

#include "ros/ros.h"

class ROSSimBehaviorServer
{
public:
    ROSSimBehaviorServer(std::unique_ptr<underwater_autonomy::Behavior> planner,
                     std::shared_ptr<ROSSimVehicleInterface> vehicleInterface);
    ROSSimBehaviorServer(ROSSimBehaviorServer&& other);
    ~ROSSimBehaviorServer() {}

    void update();

private:
    std::shared_ptr<ROSSimVehicleInterface> vehicleInterface;
    std::unique_ptr<underwater_autonomy::BehaviorController> behaviorController;

    ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif