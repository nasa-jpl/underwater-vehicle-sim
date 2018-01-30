#include "planner_framework/Action.h"

#include "planner_framework/PlanDispatcher.h"

PlanDispatcher::PlanDispatcher() :
currentAction(0),
running(false)
{}

void PlanDispatcher::runPlan()
{
	currentAction = 0;
	running = true;
}

void PlanDispatcher::setPlan(const Plan& newPlan)
{
	currentAction = 0;
	running = false;
	plan = newPlan;
}

bool PlanDispatcher::triggerReplan()
{
	if(running)
	{
		Action& action = *(plan.getActions()[currentAction]);
		if(action.getState() == Action::State::EXECUTING)
		{
			//Check the current action to see if it should trigger a replan
			return action.triggerReplan();
		}
	}
	return false;
}

void PlanDispatcher::update()
{
	if(running && currentAction < plan.getActions().size())
	{
		Action& action = *(plan.getActions()[currentAction]);
		if(action.getState() == Action::State::PLANNED) //execute the next action
		{
			action.execute();
		}
		else if(action.getState() == Action::State::EXECUTING) //moniter the current action
		{
			action.monitor();
			
		}
		else if(action.getState() == Action::State::INTERRUPTED || //Move on to the next action
				action.getState() == Action::State::COMPLETED ||
				action.getState() == Action::State::FAILED)
		{
			currentAction++;
		}
	}
}