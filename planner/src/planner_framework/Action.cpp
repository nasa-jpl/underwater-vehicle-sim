#include "planner_framework/Action.h"

Action::Action() :
	state(Action::State::PLANNED)
{}

Action::Action(const Action& action) :
	state(action.state)
{}

void Action::reset()
{
	state = Action::State::PLANNED;
}

void Action::execute()
{
	state = Action::State::DISPATCHED;
	executeAction();
}

Action::State Action::getState()
{
	return state;
}

void Action::setState(Action::State newState)
{
	state = newState;
}