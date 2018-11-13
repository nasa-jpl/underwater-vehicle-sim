#ifndef PLANNER_H
#define PLANNER_H

#include <memory>

#include "planner_framework/Plan.h"

class Planner
{
public:
	Planner() {};
	virtual ~Planner() {};

	virtual std::shared_ptr<Plan> plan()=0;

protected:
    enum GoalState {RUNNING, FAILED, SUCCESS};
private:
};

#endif