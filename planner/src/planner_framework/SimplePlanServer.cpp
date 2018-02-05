#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

#include "ros/ros.h"

SimplePlanServer::SimplePlanServer(std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner) :
	planDispatcher(std::move(planDispatcher)),
	planner(std::move(planner))
{
	planDispatcher->run();
}

SimplePlanServer::SimplePlanServer(SimplePlanServer&& other) :
	planDispatcher(std::move(other.planDispatcher)),
	planner(std::move(other.planner))
{
	planDispatcher->run();
}

void SimplePlanServer::update()
{

	planDispatcher->update();

	if(planDispatcher->triggerReplan())
	{
		std::shared_ptr<Plan> newPlan = planner->plan();
		
		if(newPlan)
		{
			planDispatcher->setPlan(newPlan);
			planDispatcher->run();
		}	
	}
}