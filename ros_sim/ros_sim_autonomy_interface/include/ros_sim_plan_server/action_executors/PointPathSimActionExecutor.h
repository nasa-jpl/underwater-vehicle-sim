#ifndef POINT_PATH_SIM_ACTION_EXECUTOR_H
#define POINT_PATH_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"

#include "underwater_planner/ActionExecutor.h"
#include "vent_planner/actions/PointPathAction.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "ros_sim_autonomy_interface/PointPathRosAction.h"


class PointPathSimActionExecutor : public ActionExecutor<PointPathAction>
{
public:
    PointPathSimActionExecutor(VehicleInfo& info);
    PointPathSimActionExecutor(const PointPathSimActionExecutor& other);
    ~PointPathSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<PointPathAction> action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<PointPathAction> action) override {}

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<PointPathAction> action) override;

    void cancel(std::shared_ptr<PointPathAction> action) override;

    std::unique_ptr<ActionExecutor<PointPathAction>> clone() override;


private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);

    /**
    * Callback that occurs when the action is finished
    * @param action Action is avalible to update the internal state
    */
    void actionDone(std::shared_ptr<PointPathAction> action,
                    const actionlib::SimpleClientGoalState& state,
                    const ros_sim_autonomy_interface::PointPathRosResultConstPtr& result);

    /**
    * Callback that occurs when the action goes active
    */
    void actionActive(std::shared_ptr<PointPathAction> action);

    /**
     * Callback that occurs when feedback is recieved from the action
     * @param action Action is avalible to update the internal state
     * @param feedback Feedback pointer
     */
    void actionFeedback(std::shared_ptr<PointPathAction> action,
                        const ros_sim_autonomy_interface::PointPathRosFeedbackConstPtr& feedback);

private:
    VehicleInfo vehicleInfo;

    ros::Publisher velPub;
    ros::Subscriber poseSub;

    bool replanGoingUp;
    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;
    tf2::Vector3 lastLocation;

    actionlib::SimpleActionClient<ros_sim_autonomy_interface::PointPathRosAction> pointPathClient;
    ros_sim_autonomy_interface::PointPathRosGoal pointPathGoal;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    /**
    * Offset to apply to the currentPoint variable in the ActionLib feedback
    */
    unsigned int currentPointOffset;

    VehiclePose currentPose;
};

#endif