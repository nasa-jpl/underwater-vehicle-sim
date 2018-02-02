#include "planner_framework/Plan.h"

Plan::Plan(const Plan& other)
{
	for(auto& action : other.actions)
	{
		std::shared_ptr<Action> newAction = action->clone();
		actions.push_back(std::move(newAction));
	}
}

Plan& Plan::operator=(const Plan& other)
{
	for(auto& action : other.actions)
	{
		std::shared_ptr<Action> newAction = action->clone();
		actions.push_back(std::move(newAction));
	}

	return *this;
}

void Plan::addAction(std::shared_ptr<Action> action)
{
	actions.push_back(action);
}

void Plan::reset()
{
	for(auto& action : actions)
	{
		action.reset();
	}
}

const std::vector<std::shared_ptr<Action>>& Plan::getActions()
{
	return actions;
}