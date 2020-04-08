#ifndef FOLLOW_HEADING_SIM_ACTION_EXECUTOR_H
#define FOLLOW_HEADING_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include "underwater_autonomy/planner/ActionExecutor.h"
#include "underwater_autonomy/planner/actions/FollowHeadingAction.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_plan_server/action_executors/SimActionExecutorFactoryMethod.h"

class FollowHeadingSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::FollowHeadingAction>,
                                       public SimActionExecutorFactoryMethod<FollowHeadingSimActionExecutor, underwater_autonomy::FollowHeadingAction>
{
public:
    FollowHeadingSimActionExecutor(underwater_autonomy::FollowHeadingAction& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo);

    FollowHeadingSimActionExecutor(const FollowHeadingSimActionExecutor&&) = delete;
    FollowHeadingSimActionExecutor(const FollowHeadingSimActionExecutor&) = delete;

    FollowHeadingSimActionExecutor& operator=(FollowHeadingSimActionExecutor&& ) = delete;
    FollowHeadingSimActionExecutor& operator=(FollowHeadingSimActionExecutor& ) = delete;

    ~FollowHeadingSimActionExecutor() {}

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
    void followHeadingCompleteCallback(const std_msgs::Bool complete);

private:
    VehicleInfo vehicleInfo;

    ros::ServiceClient followHeadingClient;
    ros::Subscriber followHeadingComplete;
    
    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;
    
    tf2::Vector3 lastLocation;
    ros::Publisher velPub;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;
};

#endif