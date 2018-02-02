#ifndef VENT_SIM_ACTION_EXECUTOR_H
#define VENT_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/YoYoPointPathAction.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"
#include "vehicle_auto_control/PointPathAction.h"


class YoYoPointPathSimActionExecutor : public ActionExecutor<YoYoPointPathAction>
{
public:
	YoYoPointPathSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	YoYoPointPathSimActionExecutor(const YoYoPointPathSimActionExecutor& other);
	~YoYoPointPathSimActionExecutor() {}

	/**
	* Executes the yoyo action in the ros simulation with the given parameters
	*/
	bool execute(std::shared_ptr<YoYoPointPathAction> action) override;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	* All monitoring is done with action callbacks so this method is not used here
	*/
	void monitor(std::shared_ptr<YoYoPointPathAction> action) {}

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	bool triggerReplan(std::shared_ptr<YoYoPointPathAction> action) override;




private:
	bool hasPublisher(std::string topic);

	/**
	* Callback that occurs when the action is finished
	* @param action Action is avalible to update the internal state
	*/
	void actionDone(std::shared_ptr<YoYoPointPathAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vehicle_auto_control::PointPathResultConstPtr& result);

	/**
	* Callback that occurs when the action goes active
	*/
  	void actionActive(std::shared_ptr<YoYoPointPathAction> action);

  	/**
  	 * Callback that occurs when feedback is recieved from the action
  	 * @param action Action is avalible to update the internal state
  	 * @param feedback Feedback pointer
  	 */
	void actionFeedback(std::shared_ptr<YoYoPointPathAction> action,
						const vehicle_auto_control::PointPathFeedbackConstPtr& feedback);

private:
	ros::NodeHandle& nh;
	ros::ServiceClient infoClient;
	underwater_vehicle_sim::GetVehicleInfo::Response vehicleInfo;
	std::unordered_map<std::string, ros::Publisher> publishers;

	std::string vehicleName;

	actionlib::SimpleActionClient<vehicle_auto_control::PointPathAction> pointPathClient;
	vehicle_auto_control::PointPathGoal pointPathGoal;
};

#endif