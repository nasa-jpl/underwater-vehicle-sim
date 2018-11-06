#include <memory>

#include "planner_framework/Action.h"
#include "planner_framework/PlanDispatcher.h"

PlanDispatcher::PlanDispatcher() :
currentAction(0),
running(false)
{}

void PlanDispatcher::run()
{
	running = true;
}

void PlanDispatcher::stop()
{
	running = false;
}

void PlanDispatcher::setPlan(std::shared_ptr<Plan> newPlan)
{
	if(newPlan && newPlan != plan)
	{
		if(running)
		{
			running = false;
			if(plan && currentAction < plan->getActions().size())
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
		if(plan)
		{
			//replan because the current plan is finished 
			if(currentAction >= plan->getActions().size())
			{
				return true;
			}
			else
			{
				std::shared_ptr<Action> action = plan->getActions()[currentAction];
				if(action->getState() == Action::State::EXECUTING)
				{
					//Check the current action to see if it should trigger a replan
					return action->triggerReplan();
				}
			}
		}
		else //No plan is present so replan to get a new one
		{
			return true;
		}
		
	}

	return false;
}

void PlanDispatcher::update()
{
	if(running && plan && currentAction < plan->getActions().size())
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
		else if(action->getState() == Action::State::COMPLETED || //Move on to the next action
				action->getState() == Action::State::FAILED)
		{
			currentAction++;
		}
	}
}