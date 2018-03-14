#include <memory>

#include "vent_planner/actions/DataTransferAction.h"
#include "planner_framework/ActionExecutor.h"


DataTransferAction::DataTransferAction(std::unique_ptr<ActionExecutor<DataTransferAction>> executor) :
	executor(std::move(executor))
{}

DataTransferAction::DataTransferAction(const DataTransferAction& action) :
	Action(action),
	executor(action.executor->clone())
{}

std::shared_ptr<Action> DataTransferAction::clone() const
{
	std::shared_ptr<Action> a(new DataTransferAction(*this));
	return a;
}

void DataTransferAction::executeAction()
{
	bool success = executor->execute(shared_from_this());

	if(!success)
	{
		state = Action::State::FAILED;
	}
}

bool DataTransferAction::triggerReplan()
{
	return executor->triggerReplan(shared_from_this());
}

void DataTransferAction::monitor()
{
	executor->monitor(shared_from_this());
}

void DataTransferAction::reset()
{
	state = Action::State::PLANNED;
}

void DataTransferAction::cancel()
{
	executor->cancel(shared_from_this());
	state = Action::State::INTERRUPTED;
}

