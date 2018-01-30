#ifndef PLAN_DISPATCHER_H
#define PLAN_DISPATCHER_H

#include "planner_framework/Plan.h"

class PlanDispatcher
{
public:
	PlanDispatcher();
	~PlanDispatcher() {}

	void runPlan();
	void setPlan(const Plan& plan);

	void update();
	bool triggerReplan();

private:
	Plan plan;
	bool running;
	unsigned int currentAction;
};

#endif