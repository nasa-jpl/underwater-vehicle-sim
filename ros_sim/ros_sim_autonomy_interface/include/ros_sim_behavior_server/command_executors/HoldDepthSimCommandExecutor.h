#ifndef HOLD_DEPTH_SIM_COMMAND_EXECUTOR_H
#define HOLD_DEPTH_SIM_COMMAND_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include "underwater_vehicle_msgs/PropulsionControllerState.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "underwater_autonomy/behaviors/CommandExecutor.h"
#include "underwater_autonomy/behaviors/commands/HoldDepthCommand.h"
#include "underwater_autonomy/util/VehiclePose.h"

#include "ros_sim_behavior_server/command_executors/SimCommandExecutorFactoryMethod.h"

class HoldDepthSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::HoldDepthCommand>,
                                   public SimCommandExecutorFactoryMethod<HoldDepthSimCommandExecutor, underwater_autonomy::HoldDepthCommand>
{
public:
    HoldDepthSimCommandExecutor(underwater_autonomy::HoldDepthCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo);

    HoldDepthSimCommandExecutor(const HoldDepthSimCommandExecutor&&) = delete;
    HoldDepthSimCommandExecutor(const HoldDepthSimCommandExecutor&) = delete;

    HoldDepthSimCommandExecutor& operator=(HoldDepthSimCommandExecutor&& ) = delete;
    HoldDepthSimCommandExecutor& operator=(HoldDepthSimCommandExecutor& ) = delete;

    ~HoldDepthSimCommandExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    void execute() override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor() override;

    void stop() override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);

    bool doubleEq(double d1, double d2);

private:
    VehicleInfo vehicleInfo;

    ros::ServiceClient goToZClient;
    ros::Subscriber propState;
    bool propStateSetup;

    tf2::Vector3 lastLocation;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;
};

#endif