#include "vent_planner/VentPlanner.h"

#include "vent_planner/VentActionExecutor.h"

VentPlanner::VentPlanner(std::unique_ptr<VentActionExecutor> executor) :
	executor(std::move(executor))
{}

Plan VentPlanner::plan()
{
	Plan plan;
	return plan;
}