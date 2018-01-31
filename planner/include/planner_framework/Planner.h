#ifndef PLANNER_H
#define PLANNER_H

#include "planner_framework/Plan.h"

class Planner
{
public:
	Planner() {}
	~Planner() {}

	virtual Plan plan()=0;

private:
};

#endif