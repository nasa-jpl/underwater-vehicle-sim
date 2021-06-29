#ifndef YOYO_SIM_COMMAND_EXECUTOR_H
#define YOYO_SIM_COMMAND_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/planner/CommandExecutor.h"
#include "underwater_autonomy/planner/commands/YoYoCommand.h"

#include "underwater_autonomy/util/VehiclePose.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_behavior_server/command_executors/SimCommandExecutorFactoryMethod.h"

class YoYoSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::YoYoCommand>,
                              public SimCommandExecutorFactoryMethod<YoYoSimCommandExecutor, underwater_autonomy::YoYoCommand>
{
public:
    YoYoSimCommandExecutor(underwater_autonomy::YoYoCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo);

    YoYoSimCommandExecutor(const YoYoSimCommandExecutor&&) = delete;
    YoYoSimCommandExecutor(const YoYoSimCommandExecutor&) = delete;

    YoYoSimCommandExecutor& operator=(YoYoSimCommandExecutor&& ) = delete;
    YoYoSimCommandExecutor& operator=(YoYoSimCommandExecutor& ) = delete;

    ~YoYoSimCommandExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    void execute() override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor() override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan() override;

    void stop() override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);
    void propStateCallback(const underwater_vehicle_msgs::PropulsionControllerState state);
    void waitForPropStateSetup();
    bool sendNewGoToZGoal();

    bool doubleEq(double d1, double d2);

private:
    VehicleInfo vehicleInfo;

    ros::ServiceClient goToZClient;
    ros::Subscriber propStateSub;
    
    bool replanNextUpdate;
    double lastReplanTime;
    double distanceSinceReplan;

    tf2::Vector3 lastLocation;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;

    bool statePropSetup;
    long prevZSeqNum;
};

#endif