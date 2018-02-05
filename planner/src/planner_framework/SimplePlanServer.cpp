#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

SimplePlanServer::SimplePlanServer(std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner) :
	planDispatcher(std::move(planDispatcher)),
	planner(std::move(planner))
{}

SimplePlanServer::SimplePlanServer(SimplePlanServer&& other) :
	planDispatcher(std::move(other.planDispatcher)),
	planner(std::move(other.planner))
{}

void SimplePlanServer::update()
{
	planDispatcher->update();

	if(planDispatcher->triggerReplan())
	{
		std::shared_ptr newPlan = planner->plan();
		
		if(newPlan)
		{
			planDispatcher->setPlan();
			planDispatcher->runPlan();
		}	
	}
}