#ifndef PLANNER_H
#define PLANNER_H

#include <memory>

#include "planner_framework/Plan.h"

class Planner
{
public:
	Planner() {}
	~Planner() {}

	virtual std::shared_ptr<Plan> plan()=0;

    /**
    * True when planning is completed and no more plans will be produced.
    * Used primarily to end the simulation
    */
    virtual bool isDone()=0;

private:
};

#endif