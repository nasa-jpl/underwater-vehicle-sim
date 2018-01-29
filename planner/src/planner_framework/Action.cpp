#include "planner_framework/Action.h"


void Action::reset()
{
	state = Action::State::planned;
}

Action::State Action::getState()
{
	return state;
}