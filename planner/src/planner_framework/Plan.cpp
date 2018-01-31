#include "planner_framework/Plan.h"

Plan::Plan(const Plan& other)
{
	for(auto& action : other.actions)
	{
		std::unique_ptr<Action> newAction = action->clone();
		actions.push_back(std::move(newAction));
	}
}

Plan& Plan::operator=(const Plan& other)
{
	for(auto& action : other.actions)
	{
		std::unique_ptr<Action> newAction = action->clone();
		actions.push_back(std::move(newAction));
	}

	return *this;
}

void Plan::addAction(std::unique_ptr<Action> action)
{
	actions.push_back(std::move(action));
}

void Plan::reset()
{
	for(auto& action : actions)
	{
		action.reset();
	}
}

const std::vector<std::unique_ptr<Action>>& Plan::getActions()
{
	return actions;
}