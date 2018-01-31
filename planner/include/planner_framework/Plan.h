#ifndef PLAN_H
#define PLAN_H

#include <vector>
#include <memory>

#include "planner_framework/Action.h"

class Plan
{

public:
	Plan() {}
	Plan(const Plan& other);
	~Plan() {}

	Plan& operator=(const Plan& other);

	/**
	* Adds an action to the end of the plan
	* @action Action to add to the plan.
	*/
	void addAction(std::unique_ptr<Action> action);

	/**
	* Get the list of actions that make up the plan
	*/

	const std::vector<std::unique_ptr<Action>>& getActions();
	
	/**
	* Resets the plan as if it has not been executed
	*/
	void reset();
private:
	std::vector<std::unique_ptr<Action>> actions;
};

#endif