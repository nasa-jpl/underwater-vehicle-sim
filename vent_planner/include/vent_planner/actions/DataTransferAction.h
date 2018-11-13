#ifndef DATA_ACTION_H
#define DATA_ACTION_H

#include <vector>
#include <memory>

#include "planner_framework/Action.h"
#include "planner_framework/ActionExecutor.h"

class DataTransferAction : public Action, public std::enable_shared_from_this<DataTransferAction>
{
public:
	DataTransferAction(std::unique_ptr<ActionExecutor<DataTransferAction>> executor);

	DataTransferAction(const DataTransferAction& action);

	~DataTransferAction() {}

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
	std::unique_ptr<ActionExecutor<DataTransferAction>> executor;

};

#endif
