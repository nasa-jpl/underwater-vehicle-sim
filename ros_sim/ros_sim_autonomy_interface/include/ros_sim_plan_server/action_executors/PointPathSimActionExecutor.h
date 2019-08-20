#ifndef POINT_PATH_SIM_ACTION_EXECUTOR_H
#define POINT_PATH_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"

#include "underwater_autonomy/planner/ActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/GoToXYRosAction.h"


class PointPathSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::PointPathAction>
{
public:
    PointPathSimActionExecutor(VehicleInfo& info);
    PointPathSimActionExecutor(ros::NodeHandle nh, VehicleInfo& info);
    PointPathSimActionExecutor(const PointPathSimActionExecutor&&) = delete;
	PointPathSimActionExecutor(const PointPathSimActionExecutor&) = delete;

	PointPathSimActionExecutor& operator=(PointPathSimActionExecutor&& ) = delete;
	PointPathSimActionExecutor& operator=(PointPathSimActionExecutor& ) = delete;

    ~PointPathSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<underwater_autonomy::PointPathAction> action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<underwater_autonomy::PointPathAction> action) override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<underwater_autonomy::PointPathAction> action) override;

    void cancel(std::shared_ptr<underwater_autonomy::PointPathAction> action) override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);

    /**
    * Callback that occurs when the action is finished
    * @param action Action is avalible to update the internal state
    */
    void actionDone(std::shared_ptr<underwater_autonomy::PointPathAction> action,
                    const actionlib::SimpleClientGoalState& state,
                    const vehicle_auto_control::GoToXYRosResultConstPtr& result);

    /**
    * Callback that occurs when the action goes active
    */
    void actionActive(std::shared_ptr<underwater_autonomy::PointPathAction> action);

    /**
     * Callback that occurs when feedback is recieved from the action
     * @param action Action is avalible to update the internal state
     * @param feedback Feedback pointer
     */
    void actionFeedback(std::shared_ptr<underwater_autonomy::PointPathAction> action,
                        const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback);

    void sendNextGoToXYGoal(std::shared_ptr<underwater_autonomy::PointPathAction> action);
private:
    VehicleInfo vehicleInfo;

    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient;

    bool replanNextUpdate;

    ros::Time lastReplan;
    double distanceSinceReplan;

    ros::Time lastUpdate;
    ros::Duration currentDuration;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    ros::Publisher velPub;
    ros::Subscriber poseSub;

    tf2::Vector3 lastLocation;
    
    underwater_autonomy::VehiclePose currentPose;
};

#endif