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

	/**
	* Adds an action to the end of the plan
	* @action Action to add to the plan.
	*/
	void addAction(std::shared_ptr<Action> action);

	/**
	* Get the list of actions that make up the plan
	*/

	const std::vector<std::shared_ptr<Action>>& getActions();
	
	/**
	* Resets the plan as if it has not been executed
	*/
	void reset();

	/**
	* Resets the interrupted actions to be continued
	*/
	void resetInterrupted();

	unsigned int getNextAction();

	/**
	 * Checks if the plan is finished. A finished plan checks if all
	 * actions were completed, either successfully or failed.
	 * @return If the plan has been completed or not
	 */
	bool isCompleted();

private:
	std::vector<std::shared_ptr<Action>> actions;
};

#endif