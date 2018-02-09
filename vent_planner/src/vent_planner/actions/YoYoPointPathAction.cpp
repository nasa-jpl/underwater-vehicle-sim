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
  	points(points),
  	currentPoint(0)
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

std::shared_ptr<Action> YoYoPointPathAction::clone() const
{
	std::shared_ptr<Action> a(new YoYoPointPathAction(*this));
	return a;
}

void YoYoPointPathAction::executeAction()
{
	bool success = executor.execute(shared_from_this());

	if(!success)
	{
		state = Action::State::FAILED;
	}
}

bool YoYoPointPathAction::triggerReplan()
{
	return executor.triggerReplan(shared_from_this());
}

void YoYoPointPathAction::monitor()
{
	executor.monitor(shared_from_this());
}

void YoYoPointPathAction::reset()
{
	currentPoint = 0;
	pointReachedTimes.clear();
	state = Action::State::PLANNED;
}

void YoYoPointPathAction::cancel()
{
	executor.cancel(shared_from_this());
	state = Action::State::INTERRUPTED;
}

void YoYoPointPathAction::setCurrentPoint(const int point)
{
	currentPoint = point;
}

const int YoYoPointPathAction::getCurrentPoint()
{
	return currentPoint;
}

void YoYoPointPathAction::setGoingUp(const bool goingUp)
{
    this->goingUp = goingUp;
}

const bool YoYoPointPathAction::getGoingUp()
{
    return goingUp;
}

void YoYoPointPathAction::addPointReachedTime(const ros::Time& time)
{
	pointReachedTimes.push_back(time);
}

const std::vector<ros::Time>& YoYoPointPathAction::getPointReachedTimes()
{
	return pointReachedTimes;
}