#ifndef HOLD_DEPTH_SIM_ACTION_EXECUTOR_H
#define HOLD_DEPTH_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"

#include "underwater_autonomy/planner/ActionExecutor.h"
#include "underwater_autonomy/planner/actions/HoldDepthAction.h"

#include "underwater_autonomy/util/VehiclePose.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/GoToZRosAction.h"


class HoldDepthSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::HoldDepthAction>
{
public:
    HoldDepthSimActionExecutor(VehicleInfo& info);
    HoldDepthSimActionExecutor(ros::NodeHandle nh, VehicleInfo& vehicleInfo);

    HoldDepthSimActionExecutor(const HoldDepthSimActionExecutor&&) = delete;
	HoldDepthSimActionExecutor(const HoldDepthSimActionExecutor&) = delete;

	HoldDepthSimActionExecutor& operator=(HoldDepthSimActionExecutor&& ) = delete;
	HoldDepthSimActionExecutor& operator=(HoldDepthSimActionExecutor& ) = delete;

    ~HoldDepthSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<underwater_autonomy::HoldDepthAction> action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<underwater_autonomy::HoldDepthAction> action) override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<underwater_autonomy::HoldDepthAction> action) override;

    void cancel(std::shared_ptr<underwater_autonomy::HoldDepthAction> action) override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);

    /**
    * Callback that occurs when the action is finished
    * @param action Action is avalible to update the internal state
    */
    void rosActionDone(std::shared_ptr<underwater_autonomy::HoldDepthAction> action,
                    const actionlib::SimpleClientGoalState& state,
                    const vehicle_auto_control::GoToZRosResultConstPtr& result);

    /**
    * Callback that occurs when the action goes active
    */
    void rosActionActive(std::shared_ptr<underwater_autonomy::HoldDepthAction> action);

    /**
     * Callback that occurs when feedback is recieved from the action
     * @param action Action is avalible to update the internal state
     * @param feedback Feedback pointer
     */
    void rosActionFeedback(std::shared_ptr<underwater_autonomy::HoldDepthAction> action,
                        const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback);

private:
    VehicleInfo vehicleInfo;

    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient;
    vehicle_auto_control::GoToZRosGoal goToZGoal;


    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;
    
    ros::Time lastUpdate;
    ros::Duration currentDuration;
    underwater_autonomy::Action::State stateAfterCancel;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    tf2::Vector3 lastLocation;
    ros::Publisher velPub;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;
};

#endif