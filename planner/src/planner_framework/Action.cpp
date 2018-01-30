#include "planner_framework/Action.h"


void Action::reset()
{
	state = Action::State::PLANNED;
}

Action::State Action::getState()
{
	return state;
}