#include <memory>

#include "planner_framework/Action.h"
#include "planner_framework/PlanDispatcher.h"

PlanDispatcher::PlanDispatcher() :
currentAction(0),
running(false)
{}

void PlanDispatcher::runPlan()
{
	running = true;
}

void PlanDispatcher::setPlan(std::shared_ptr<Plan> newPlan)
{
	if(newPlan && newPlan != plan)
	{
		if(running)
		{
			running = false;
			if(currentAction < plan->getActions().size())
			{
				plan->getActions()[currentAction]->cancel();
			}
		}
		
		plan = newPlan;
		currentAction = plan->getNextAction();
	}
}

bool PlanDispatcher::triggerReplan()
{
	if(running)
	{
		std::shared_ptr<Action> action = plan->getActions()[currentAction];
		if(action->getState() == Action::State::EXECUTING)
		{
			//Check the current action to see if it should trigger a replan
			return action->triggerReplan();
		}

		//replan because the current plan is finished 
		if(currentAction == plan->getActions().size())
		{
			return true;
		}
	}
	return false;
}

void PlanDispatcher::update()
{
	if(running && currentAction < plan->getActions().size())
	{
		std::shared_ptr<Action> action = plan->getActions()[currentAction];
		if(action->getState() == Action::State::PLANNED) //execute the next action
		{
			action->execute();
		}
		else if(action->getState() == Action::State::DISPATCHED ||
				action->getState() == Action::State::EXECUTING) //moniter the current action
		{
			action->monitor();
			
		}
		else if(action->getState() == Action::State::INTERRUPTED || //Move on to the next action
				action->getState() == Action::State::COMPLETED ||
				action->getState() == Action::State::FAILED)
		{
			currentAction++;
		}
	}
}