#include "planner_framework/Plan.h"

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