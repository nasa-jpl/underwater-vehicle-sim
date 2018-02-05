#include "planner_framework/Plan.h"

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

void Plan::resetInterrupted()
{
	for(auto& action : actions)
	{
		if(action->getState() == Action::State::INTERRUPTED)
		{
			action->setState(Action::State::PLANNED);
		}
	}
}

const std::vector<std::shared_ptr<Action>>& Plan::getActions()
{
	return actions;
}

unsigned int Plan::getNextAction()
{
	for(unsigned int i = 0; i <  actions.size(); i++)
	{
		auto action = actions[i];
		if(action->getState() == Action::State::PLANNED)
		{
			return i;
		}
	}
	return actions.size();
}