#ifndef CHARGE_SIM_ACTION_EXECUTOR_H
#define CHARGE_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "planner_framework/ActionExecutor.h"
#include "vent_planner/actions/ChargeAction.h"

#include "underwater_vehicle_sim/GetVehicleInfo.h"

#include "actionlib/client/simple_action_client.h"


class ChargeSimActionExecutor : public ActionExecutor<ChargeAction>
{
public:
	ChargeSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName);
	ChargeSimActionExecutor(const ChargeSimActionExecutor& other);
	~ChargeSimActionExecutor() {}

	/**
	* Executes the yoyo action in the ros simulation with the given parameters
	*/
	bool execute(std::shared_ptr<ChargeAction> action) override;
	
	/**
	* Monitors and updates the state of the yoyo action in the ros simulation 
	* All monitoring is done with action callbacks so this method is not used here
	*/
	void monitor(std::shared_ptr<ChargeAction> action) {}

	/**
	* Allows the yoyo action to trigger a replan in the ros simulation 
	*/
	bool triggerReplan(std::shared_ptr<ChargeAction> action) override;

	void cancel(std::shared_ptr<ChargeAction> action) override;




private:
	bool hasPublisher(std::string topic);

	/**
	* Callback that occurs when the action is finished
	* @param action Action is avalible to update the internal state
	*/
	void actionDone(std::shared_ptr<ChargeAction> action,
					const actionlib::SimpleClientGoalState& state);

	/**
	* Callback that occurs when the action goes active
	*/
  	void actionActive(std::shared_ptr<ChargeAction> action);

  	/**
  	 * Callback that occurs when feedback is recieved from the action
  	 * @param action Action is avalible to update the internal state
  	 * @param feedback Feedback pointer
  	 */
	void actionFeedback(std::shared_ptr<ChargeAction> action);

private:
	ros::NodeHandle& nh;
	ros::ServiceClient infoClient;
	underwater_vehicle_sim::GetVehicleInfo::Response vehicleInfo;
	std::unordered_map<std::string, ros::Publisher> publishers;

	std::string vehicleName;

};

#endif
