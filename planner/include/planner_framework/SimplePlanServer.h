#ifndef SIMPLE_PLAN_SERVER_H
#define SIMPLE_PLAN_SERVER_H

#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

class SimplePlanServer
{
public:
	SimplePlanServer(PlanDispatcher planDispatcher, Planner& planner);
	~SimplePlanServer() {}

	void update();

private:
	PlanDispatcher planDispatcher;
	Planner& planner;
};

#endif