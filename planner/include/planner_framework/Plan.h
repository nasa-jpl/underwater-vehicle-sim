#ifndef PLAN_H
#define PLAN_H

#include <vector>
#include <memory>

#include "planner_framework/Action.h"

class Plan
{
public:
	Plan() {}
	~Plan() {}

	void addAction(std::unique_ptr<Action> action);
	const std::vector<std::unique_ptr<Action>>& getActions();
	
	void reset();
private:
	std::vector<std::unique_ptr<Action>> actions;
};

#endif