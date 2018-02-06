#include <memory>

#include "vent_planner/actions/ChargeAction.h"
#include "planner_framework/ActionExecutor.h"


ChargeAction::ChargeAction(ActionExecutor<ChargeAction>& executor) :
	executor(executor)
{}

ChargeAction::ChargeAction(const ChargeAction& action) :
	Action(action),
	executor(action.executor)
{}

std::shared_ptr<Action> ChargeAction::clone() const
{
	std::shared_ptr<Action> a(new ChargeAction(*this));
	return a;
}

void ChargeAction::executeAction()
{
	bool success = executor.execute(shared_from_this());

	if(!success)
	{
		state = Action::State::FAILED;
	}
}

bool ChargeAction::triggerReplan()
{
	return executor.triggerReplan(shared_from_this());
}

void ChargeAction::monitor()
{
	executor.monitor(shared_from_this());
}

void ChargeAction::reset()
{
	state = Action::State::PLANNED;
}

void ChargeAction::cancel()
{
	executor.cancel(shared_from_this());
	state = Action::State::INTERRUPTED;
}

