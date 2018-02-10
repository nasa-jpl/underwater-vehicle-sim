#ifndef SIMPLE_PLAN_SERVER_H
#define SIMPLE_PLAN_SERVER_H

#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

class SimplePlanServer
{
public:
	SimplePlanServer(std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner);
	SimplePlanServer(SimplePlanServer&& other);
	~SimplePlanServer() {}

	void update();

    /**
    * True when planning is completed and no more plans will be produced.
    * Used primarily to end the simulation
    */
    bool isDone();

private:
	std::unique_ptr<PlanDispatcher> planDispatcher;
	std::unique_ptr<Planner> planner;
};

#endif