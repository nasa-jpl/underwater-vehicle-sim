#ifndef VENT_PLANNER_H
#define VENT_PLANNER_H

#include <vector>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"

class VentPlanner : public Planner
{
public:
	VentPlanner();
	~VentPlanner() {}

	Plan plan();

	static std::vector<tf::Vector3> makeSpiral(tf::Vector3 startLocation, 
									 	double startDirection, 
									 	double spacing, 
									 	double size);

	static std::vector<tf::Vector3> makeLawnmower(const tf::Vector3& startLocation,
										   		  double alongTrackDirection,
										   		  double acrossTrackDirection,
										   		  double alongTrackSize,
										   		  double acrossTrackSize,
		 								   		  double spacing);

private:

	
};

#endif