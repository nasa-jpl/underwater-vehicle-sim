#ifndef POINT_PATH_SIM_ACTION_EXECUTOR_H
#define POINT_PATH_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "underwater_planner/ActionExecutor.h"
#include "vent_planner/actions/PointPathAction.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "ros_sim_plan_server/PointPathRosAction.h"


class PointPathSimActionExecutor : public ActionExecutor<PointPathAction>
{
public:
    PointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
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
    bool hasPublisher(std::string topic);

    /**
    * Callback that occurs when the action is finished
    * @param action Action is avalible to update the internal state
    */
    void actionDone(std::shared_ptr<PointPathAction> action,
                    const actionlib::SimpleClientGoalState& state,
                    const ros_sim_plan_server::PointPathRosResultConstPtr& result);

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
                        const ros_sim_plan_server::PointPathRosFeedbackConstPtr& feedback);

private:
    ros::NodeHandle& nh;
    ros::ServiceClient infoClient;
    underwater_vehicle_msgs::GetVehicleInfo::Response vehicleInfo;
    std::unordered_map<std::string, ros::Publisher> publishers;

    bool replanGoingUp;
    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;
    tf::Vector3 lastLocation;
    std::string vehicleName;

    actionlib::SimpleActionClient<ros_sim_plan_server::PointPathRosAction> pointPathClient;
    ros_sim_plan_server::PointPathRosGoal pointPathGoal;

    tf::TransformListener listener;

    /**
    * Offset to apply to the currentPoint variable in the ActionLib feedback
    */
    unsigned int currentPointOffset;
};

#endif