#ifndef CIRCLE_SIM_COMMAND_EXECUTOR_H
#define CIRCLE_SIM_COMMAND_EXECUTOR_H

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
#include "underwater_autonomy/util/VehiclePose.h"
#include "underwater_autonomy/planner/commands/CircleCommand.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_plan_server/command_executors/SimCommandExecutorFactoryMethod.h"

class CircleSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::CircleCommand>,
                                   public SimCommandExecutorFactoryMethod<CircleSimCommandExecutor, underwater_autonomy::CircleCommand>
{
public:
    CircleSimCommandExecutor(underwater_autonomy::CircleCommand& action, ros::NodeHandle& nh, VehicleInfo& info);
    CircleSimCommandExecutor(const CircleSimCommandExecutor&&) = delete;
    CircleSimCommandExecutor(const CircleSimCommandExecutor&) = delete;

    CircleSimCommandExecutor& operator=(CircleSimCommandExecutor&& ) = delete;
    CircleSimCommandExecutor& operator=(CircleSimCommandExecutor& ) = delete;

    ~CircleSimCommandExecutor() {}

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
    void createCirclePoints();


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

    std::vector<Eigen::Vector2d> circlePoints;
    uint currentCirclePoint = 0;
};

#endif