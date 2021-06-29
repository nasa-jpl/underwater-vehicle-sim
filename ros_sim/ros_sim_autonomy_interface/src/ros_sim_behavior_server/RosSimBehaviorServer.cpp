#include "ros_sim_behavior_server/ROSSimBehaviorServer.h"

#include "std_msgs/Float64.h"

#include "ros/ros.h"

using namespace underwater_autonomy;

ROSSimBehaviorServer::ROSSimBehaviorServer(std::unique_ptr<Behavior> behavior,
                                           std::shared_ptr<ROSSimVehicleInterface> vehicleInterface) :
    vehicleInterface(vehicleInterface),
    behaviorController(new BehaviorController(std::move(behavior), vehicleInterface))
{
    ros::NodeHandle nh;

    clockSpeedPub = nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1);
    nh.param<float>("/speed_up_factor", speedUpFactor, 1);
    
    behaviorController->start();
}

ROSSimBehaviorServer::ROSSimBehaviorServer(ROSSimBehaviorServer&& other) :
    vehicleInterface(other.vehicleInterface),
    behaviorController(std::move(other.behaviorController))
{
    behaviorController->start();
}

void ROSSimBehaviorServer::update()
{
    std_msgs::Float64 slowSim;
    slowSim.data = 1;
    clockSpeedPub.publish(slowSim);

    behaviorController->updateBehavior();

    std_msgs::Float64 startSim;
    startSim.data = speedUpFactor;
    clockSpeedPub.publish(startSim);

    vehicleInterface->updateCommands();
    if(behaviorController->getBehaviorState() == Behavior::ExecutionState::RUNNING) {
        vehicleInterface->handleRequestedCommands();
    }
}