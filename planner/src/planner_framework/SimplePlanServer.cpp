#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

SimplePlanServer::SimplePlanServer(PlanDispatcher planDispatcher, Planner& planner) :
	planDispatcher(planDispatcher),
	planner(planner)
{}

void SimplePlanServer::update()
{
	planDispatcher.update();

	if(planDispatcher.triggerReplan())
	{
		planDispatcher.setPlan(planner.plan());
		planDispatcher.runPlan();
	}
}