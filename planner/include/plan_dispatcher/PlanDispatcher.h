#ifndef PLAN_DISPATCHER_H
#define PLAN_DISPATCHER_H

#include "planner_framework/Plan.h"

class PlanDispatcher
{
public:
	PlanDispatcher();
	~PlanDispatcher() {}

	/**
	* Run the current plan
	*/
	void runPlan();

	/**
	* Sets the plan to be executed. Stops current plan.
	*/
	void setPlan(const Plan& plan);

	/**
	* Updates the state of the current action and the overall plan
	*/
	void update();

	/**
	* Allows the plan to trigger a replan.  Returns true if a replan is requested
	*/
	bool triggerReplan();

private:
	Plan plan;
	bool running;
	unsigned int currentAction;
};

#endif