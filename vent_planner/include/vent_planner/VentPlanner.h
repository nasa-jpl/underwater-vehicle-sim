#ifndef VENT_PLANNER_H
#define VENT_PLANNER_H

#include "planner_framework/Planner.h"

#include "vent_planner/VentActionExecutor.h"

class VentPlanner : public Planner
{
public:
	VentPlanner(std::unique_ptr<VentActionExecutor> executor);
	~VentPlanner() {}

	Plan plan();

private:
	std::unique_ptr<VentActionExecutor> executor;
};

#endif