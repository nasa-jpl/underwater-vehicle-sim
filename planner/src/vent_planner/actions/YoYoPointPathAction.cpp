#include <memory>

#include "vent_planner/VentActionExecutor.h"

#include "vent_planner/actions/YoYoPointPathAction.h"

YoYoPointPathAction::YoYoPointPathAction(std::shared_ptr<VentActionExecutor> executor, 
										 const std::string vehicleName, 
										 const double targetHorizontalVelocity, 
										 const double targetRotationalVelocity,
										 const double targetSlope, 
										 const double minDepth,
										 const double maxDepth,
										 std::vector<tf::Vector3>& points) :
  executor(executor),
  vehicleName(vehicleName),
  targetHorizontalVelocity(targetHorizontalVelocity),
  targetRotationalVelocity(targetRotationalVelocity),
  targetSlope(targetSlope),
  minDepth(minDepth),
  maxDepth(maxDepth),
  points(points)
{}

void YoYoPointPathAction::execute() 
{
	executor->executeYoYoPointPathAction(vehicleName, 
										 targetHorizontalVelocity, 
										 targetRotationalVelocity, 
										 targetSlope, 
										 minDepth, 
										 maxDepth, 
										 points);
}

bool YoYoPointPathAction::tiggerReplan() 
{
	return false;
}

void YoYoPointPathAction::updateState()
{
}