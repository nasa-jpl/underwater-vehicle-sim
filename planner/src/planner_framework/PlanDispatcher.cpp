#include <memory>

#include "ros/ros.h"

#include "planner_framework/Action.h"
#include "planner_framework/PlanDispatcher.h"

PlanDispatcher::PlanDispatcher() :
currentAction(0),
running(false)
{}

void PlanDispatcher::run()
{
	ROS_INFO("Run PlanDispatcher");
	running = true;
}

void PlanDispatcher::stop()
{
	running = false;
}

void PlanDispatcher::setPlan(std::shared_ptr<Plan> newPlan)
{
	ROS_INFO("PlanDispatcher: setPlan Start.");
	if(newPlan && newPlan != plan)
	{
		ROS_INFO("PlanDispatcher: Check for running.");
		if(running)
		{
			ROS_INFO("PlanDispatcher: Stop runnning");
			running = false;
			if(plan && currentAction < plan->getActions().size())
			{
				ROS_INFO("PlanDispatcher: Cancel previous action.");
				plan->getActions()[currentAction]->cancel();
			}
		}
		
		plan = newPlan;
		currentAction = plan->getNextAction();
		ROS_INFO("PlanDispatcher: Set new plan, Start on action: %i", currentAction);
	}
}

bool PlanDispatcher::triggerReplan()
{
	if(running)
	{
		if(plan)
		{
			//replan because the current plan is finished 
			if(currentAction == plan->getActions().size())
			{
				ROS_INFO("PlanDispatcher: At end of plan, replan");
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
			ROS_INFO("PlanDispatcher: Execute action");
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
			ROS_INFO("PlanDispatcher: Next Action: %i", currentAction);
		}
	}
}