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

class FollowHeadingSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::FollowHeadingAction>
{
public:
    FollowHeadingSimActionExecutor(VehicleInfo& info);
    FollowHeadingSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo);

    FollowHeadingSimActionExecutor(const FollowHeadingSimActionExecutor&&) = delete;
	FollowHeadingSimActionExecutor(const FollowHeadingSimActionExecutor&) = delete;

	FollowHeadingSimActionExecutor& operator=(FollowHeadingSimActionExecutor&& ) = delete;
	FollowHeadingSimActionExecutor& operator=(FollowHeadingSimActionExecutor& ) = delete;

    ~FollowHeadingSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<underwater_autonomy::FollowHeadingAction> action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<underwater_autonomy::FollowHeadingAction> action) override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<underwater_autonomy::FollowHeadingAction> action) override;

    void cancel(std::shared_ptr<underwater_autonomy::FollowHeadingAction> action) override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);
    void followHeadingCompleteCallback(const std_msgs::Bool complete);

private:
    VehicleInfo vehicleInfo;

    ros::Publisher followHeadingPub;
	ros::Publisher followHeadingEnablePub;
	ros::Subscriber followHeadingComplete;

    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;
    
    ros::Time lastUpdate;
    ros::Duration currentDuration;

    tf2::Vector3 lastLocation;
    ros::Publisher velPub;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;

    bool gotCompleteCallback;
};

#endif