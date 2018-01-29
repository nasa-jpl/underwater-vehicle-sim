#ifndef YOYO_POINT_PATH_ACTION_H
#define YOYO_POINT_PATH_ACTION_H

#include <vector>
#include <memory>
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Action.h"
#include "vent_planner/VentActionExecutor.h"

class YoYoPointPathAction : public Action
{
public:
	YoYoPointPathAction(std::shared_ptr<VentActionExecutor> executor, 
						const std::string vehicleName, 
						const double targetHorizontalVelocity, 
						const double targetRotationalVelocity,
						const double targetSlope,
						const double minDepth,
						const double maxDepth,
						std::vector<tf::Vector3>& points);
	~YoYoPointPathAction() {}

	void execute();
	bool tiggerReplan();
	void updateState();

private:
	std::shared_ptr<VentActionExecutor> executor;
	const std::string vehicleName;
	const double targetHorizontalVelocity;
	const double targetRotationalVelocity;
	const double targetSlope;
	const double minDepth;
	const double maxDepth;
	
	std::vector<tf::Vector3> points;
};

#endif