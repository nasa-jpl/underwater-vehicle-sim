#include <memory>

#include "vent_planner/actions/YoYoPointPathAction.h"
#include "planner_framework/ActionExecutor.h"

YoYoPointPathAction::YoYoPointPathAction(ActionExecutor<YoYoPointPathAction>& executor,
										 const double targetHorizontalVelocity,
										 const double targetRotationalVelocity,
										 const double targetSlope,
										 const double upperDepth,
										 const double lowerDepth,
										 const std::vector<tf::Vector3>& points) :
	executor(executor),
  	targetHorizontalVelocity(targetHorizontalVelocity),
  	targetRotationalVelocity(targetRotationalVelocity),
  	targetSlope(targetSlope),
  	upperDepth(upperDepth),
  	lowerDepth(lowerDepth),
  	points(points)
{}

YoYoPointPathAction::YoYoPointPathAction(const YoYoPointPathAction& action) :
	Action(action),
	executor(action.executor),
  	targetHorizontalVelocity(action.targetHorizontalVelocity),
  	targetRotationalVelocity(action.targetRotationalVelocity),
  	targetSlope(action.targetSlope),
  	upperDepth(action.upperDepth),
  	lowerDepth(action.lowerDepth),
  	points(action.points)
{}

std::unique_ptr<Action> YoYoPointPathAction::clone() const
{
	std::unique_ptr<Action> a(new YoYoPointPathAction(*this));
	return a;
}

void YoYoPointPathAction::executeAction()
{
	bool success = executor.execute(*this);

	if(!success)
	{
		state = Action::State::FAILED;
	}
}

bool YoYoPointPathAction::triggerReplan()
{
	return executor.triggerReplan(*this);
}

void YoYoPointPathAction::monitor()
{
	executor.monitor(*this);
}