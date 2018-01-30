#include <memory>

#include "vent_planner/VentActionExecutor.h"

#include "vent_planner/actions/YoYoPointPathAction.h"

YoYoPointPathAction::YoYoPointPathAction(VentActionExecutor& executor, 
										 const double targetHorizontalVelocity, 
										 const double targetRotationalVelocity,
										 const double targetSlope, 
										 const double upperDepth,
										 const double lowerDepth,
										 std::vector<tf::Vector3>& points) :
	executor(executor),
  	targetHorizontalVelocity(targetHorizontalVelocity),
  	targetRotationalVelocity(targetRotationalVelocity),
  	targetSlope(targetSlope),
  	upperDepth(upperDepth),
  	lowerDepth(lowerDepth),
  	points(points)
{}

void YoYoPointPathAction::execute()
{
	executor.executeYoYoPointPathAction(targetHorizontalVelocity, 
										 targetRotationalVelocity, 
										 targetSlope, 
										 upperDepth, 
										 lowerDepth, 
										 points);
}

bool YoYoPointPathAction::tiggerReplan() 
{
	return executor.triggerReplanYoYoPointPathAction();
}

void YoYoPointPathAction::monitor()
{
	executor.monitorYoYoPointPathAction(state);
}