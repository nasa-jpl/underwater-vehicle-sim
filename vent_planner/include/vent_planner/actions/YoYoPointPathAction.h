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
	YoYoPointPathAction(VentActionExecutor& executor, 
						const double targetHorizontalVelocity, 
						const double targetRotationalVelocity,
						const double targetSlope,
						const double upperDepth,
						const double lowerDepth,
						const std::vector<tf::Vector3>& points);

	YoYoPointPathAction(const YoYoPointPathAction& action);

	~YoYoPointPathAction() {}

	std::unique_ptr<Action> clone() const override;

	void executeAction();
	bool triggerReplan();
	void monitor();

private:
	VentActionExecutor& executor;
	const double targetHorizontalVelocity;
	const double targetRotationalVelocity;
	const double targetSlope;
	const double upperDepth;
	const double lowerDepth;
	
	std::vector<tf::Vector3> points;
};

#endif