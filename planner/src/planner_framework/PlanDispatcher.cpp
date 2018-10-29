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
	ROS_INFO("Run Plan Dispatcher");
	running = true;
}

void PlanDispatcher::stop()
{
	ROS_INFO("Stop Plan Dispatcher");
	running = false;
}

void PlanDispatcher::setPlan(std::shared_ptr<Plan> newPlan)
{
	if(newPlan && newPlan != plan)
	{
		ROS_INFO("Check for running plan");
		if(running)
		{
			ROS_INFO("Stop runnning plan");
			running = false;
			if(plan && currentAction < plan->getActions().size())
			{
				ROS_INFO("Cancel current action");
				plan->getActions()[currentAction]->cancel();
			}
		}
		
		plan = newPlan;
		currentAction = plan->getNextAction();
		ROS_INFO("Set new plan, Start on action: %i", currentAction);
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
				ROS_INFO("At end of plan, replan. Current Action: %i, Plan Size: %lu", currentAction, plan->getActions().size());
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
			ROS_INFO("Execute action %i", currentAction);
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
			ROS_INFO("Action finished, next action %i", currentAction);
		}
	}
}