#ifndef WAYPOINTS_SIM_COMMAND_EXECUTOR_H
#define WAYPOINTS_SIM_COMMAND_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "underwater_vehicle_msgs/PropulsionControllerState.h"
#include "std_msgs/Bool.h"

#include "underwater_autonomy/planner/CommandExecutor.h"
#include "underwater_autonomy/planner/commands/WaypointsCommand.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_behavior_server/command_executors/SimCommandExecutorFactoryMethod.h"

class WaypointsSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::WaypointsCommand>,
                                   public SimCommandExecutorFactoryMethod<WaypointsSimCommandExecutor, underwater_autonomy::WaypointsCommand>
{
public:
    WaypointsSimCommandExecutor(underwater_autonomy::WaypointsCommand& action, ros::NodeHandle& nh, VehicleInfo& info);
    WaypointsSimCommandExecutor(const WaypointsSimCommandExecutor&&) = delete;
    WaypointsSimCommandExecutor(const WaypointsSimCommandExecutor&) = delete;

    WaypointsSimCommandExecutor& operator=(WaypointsSimCommandExecutor&& ) = delete;
    WaypointsSimCommandExecutor& operator=(WaypointsSimCommandExecutor& ) = delete;

    ~WaypointsSimCommandExecutor() {}

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

    bool sendNextGoToXYGoal();

    bool doubleEq(double d1, double d2);
    
private:
    VehicleInfo vehicleInfo;

    ros::ServiceClient goToXYClient;
    ros::Subscriber propStateSub;
    
    bool replanNextUpdate;
    double lastReplanTime;
    double distanceSinceReplan;

    ros::Subscriber poseSub;
    
    underwater_autonomy::VehiclePose currentPose;

    bool statePropSetup;
    long prevXYSeqNum;

    bool poseAtFirstExecuteValid;
    underwater_autonomy::VehiclePose poseAtFirstExecute;
};

#endif