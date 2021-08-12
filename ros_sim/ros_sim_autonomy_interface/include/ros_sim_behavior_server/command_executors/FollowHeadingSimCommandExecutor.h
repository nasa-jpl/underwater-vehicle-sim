#ifndef FOLLOW_HEADING_SIM_COMMAND_EXECUTOR_H
#define FOLLOW_HEADING_SIM_COMMAND_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include "underwater_autonomy/planner/CommandExecutor.h"
#include "underwater_autonomy/planner/commands/FollowHeadingCommand.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_behavior_server/command_executors/SimCommandExecutorFactoryMethod.h"

class FollowHeadingSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::FollowHeadingCommand>,
                                       public SimCommandExecutorFactoryMethod<FollowHeadingSimCommandExecutor, underwater_autonomy::FollowHeadingCommand>
{
public:
    FollowHeadingSimCommandExecutor(underwater_autonomy::FollowHeadingCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo);

    FollowHeadingSimCommandExecutor(const FollowHeadingSimCommandExecutor&&) = delete;
    FollowHeadingSimCommandExecutor(const FollowHeadingSimCommandExecutor&) = delete;

    FollowHeadingSimCommandExecutor& operator=(FollowHeadingSimCommandExecutor&& ) = delete;
    FollowHeadingSimCommandExecutor& operator=(FollowHeadingSimCommandExecutor& ) = delete;

    ~FollowHeadingSimCommandExecutor() {}

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
    void followHeadingCompleteCallback(const std_msgs::Bool complete);

private:
    VehicleInfo vehicleInfo;

    ros::ServiceClient followHeadingClient;
    ros::Subscriber followHeadingComplete;
        
    tf2::Vector3 lastLocation;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;
};

#endif