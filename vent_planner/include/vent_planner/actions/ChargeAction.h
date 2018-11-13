#ifndef CHARGE_ACTION_H
#define CHARGE_ACTION_H

#include <vector>
#include <memory>

#include "planner_framework/Action.h"
#include "planner_framework/ActionExecutor.h"

class ChargeAction : public Action, public std::enable_shared_from_this<ChargeAction>
{
public:
	ChargeAction(std::unique_ptr<ActionExecutor<ChargeAction>> executor);

	ChargeAction(const ChargeAction& action);

	~ChargeAction() {}

	std::shared_ptr<Action> clone() const override;

	/**
	*Executes the action using the provided executor
	*/
	void executeAction() override;

	/**
	* Allows the action to trigger a replan
	*/
	bool triggerReplan() override;

	/**
	* Monitors the state of the action and updates it as needed
	*/
	void monitor() override;

	/**
	*Resets this action to a state as if it has not been executed.
	*/
	void reset() override;

	void cancel() override;

private:
	std::unique_ptr<ActionExecutor<ChargeAction>> executor;

};

#endif
