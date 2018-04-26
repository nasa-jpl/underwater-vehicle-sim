#ifndef DYNAMIC_LAWNMOWER_SIM_ACTION_EXECUTOR_H
#define DYNAMIC_LAWNMOWER_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "vent_planner/ExecuteDynamicLawnmowerAction.h"
#include "vehicle_auto_control/DynamicLawnmowerAction.h"

class DynamicLawnmowerSimActionExecutor : public ActionExecutor<DynamicLawnmowerAction>
{
public:
    DynamicLawnmowerSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
    DynamicLawnmowerSimActionExecutor(const DynamicLawnmowerSimActionExecutor& other);
    ~DynamicLawnmowerSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<DynamicLawnmowerAction> action) override;


    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<DynamicLawnmowerAction> action) {}

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<DynamicLawnmowerAction> action) override;

    void cancel(std::shared_ptr<DynamicLawnmowerAction> action) override;

    std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> clone() override;


private:
    /**
    * Callback that occurs when the action is finished
    * @param action Action is avalible to update the internal state
    */
    void actionDone(std::shared_ptr<DynamicLawnmowerAction> action,
                    const actionlib::SimpleClientGoalState& state,
                    const vehicle_auto_control::DynamicLawnmowerResultConstPtr& result);

    void executeAction(const vent_planner::ExecuteDynamicLawnmowerGoalConstPtr& goal,
                       actionlib::SimpleActionServer<vent_planner::ExecuteDynamicLawnmowerAction>* as);

    /**
    * Callback that occurs when the action goes active
    */
    void actionActive(std::shared_ptr<DynamicLawnmowerAction> action);

    /**
     * Callback that occurs when feedback is recieved from the action
     * @param action Action is avalible to update the internal state
     * @param feedback Feedback pointer
     */
    void actionFeedback(std::shared_ptr<DynamicLawnmowerAction> action,
                        const vehicle_auto_control::DynamicLawnmowerFeedbackConstPtr& feedback);

private:
    ros::NodeHandle& nh;
    ros::ServiceClient infoClient;
    underwater_vehicle_sim::GetVehicleInfo::Response vehicleInfo;
    ros::Publisher velPublisher;
    actionlib::SimpleActionServer<vent_planner::ExecuteDynamicLawnmowerAction> actionServer;

    bool replanNextUpdate;
    
    std::string vehicleName;

    actionlib::SimpleActionClient<vehicle_auto_control::DynamicLawnmowerAction> dynamicLawnmowerClient;
    vehicle_auto_control::DynamicLawnmowerGoal dynamicLawnmowerGoal;

    tf::TransformListener listener;

};

#endif