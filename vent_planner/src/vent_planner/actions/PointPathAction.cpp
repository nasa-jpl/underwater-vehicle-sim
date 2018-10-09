#include <memory>

#include "vent_planner/actions/PointPathAction.h"
#include "planner_framework/ActionExecutor.h"

PointPathAction::PointPathAction(std::unique_ptr<ActionExecutor<PointPathAction>> executor,
                                 const double targetHorizontalVelocity,
                                 const double targetRotationalVelocity,
                                 const double targetSlope,
                                 const double upperDepth,
                                 const double lowerDepth,
                                 const std::vector<tf::Vector3>& points,
                                 const ReplanType replanType,
                                 const double periodicReplanTime) :
    executor(std::move(executor)),
    targetHorizontalVelocity(targetHorizontalVelocity),
    targetRotationalVelocity(targetRotationalVelocity),
    targetSlope(targetSlope),
    upperDepth(upperDepth),
    lowerDepth(lowerDepth),
    yoyo(true),
    points(points),
    replanType(replanType),
    periodicReplanTime(periodicReplanTime),
    currentPoint(0),
    doInterruptPoint(false)
{}

PointPathAction::PointPathAction(std::unique_ptr<ActionExecutor<PointPathAction>> executor,
                                 const double targetHorizontalVelocity,
                                 const double targetRotationalVelocity,
                                 const double targetSlope,
                                 const std::vector<tf::Vector3>& points,
                                 const ReplanType replanType,
                                 const double periodicReplanTime) :
    executor(std::move(executor)),
    targetHorizontalVelocity(targetHorizontalVelocity),
    targetRotationalVelocity(targetRotationalVelocity),
    targetSlope(targetSlope),
    upperDepth(0),
    lowerDepth(0),
    yoyo(false),
    points(points),
    replanType(replanType),
    periodicReplanTime(periodicReplanTime),
    currentPoint(0),
    doInterruptPoint(false)
{}

PointPathAction::PointPathAction(const PointPathAction& action) :
    Action(action),
    executor(action.executor->clone()),
    targetHorizontalVelocity(action.targetHorizontalVelocity),
    targetRotationalVelocity(action.targetRotationalVelocity),
    targetSlope(action.targetSlope),
    upperDepth(action.upperDepth),
    lowerDepth(action.lowerDepth),
    yoyo(action.yoyo),
    points(action.points),
    replanType(action.replanType),
    periodicReplanTime(action.periodicReplanTime),
    doInterruptPoint(action.doInterruptPoint),
    interruptPoint(action.interruptPoint)
{}

std::shared_ptr<Action> PointPathAction::clone() const
{
    std::shared_ptr<Action> a(new PointPathAction(*this));
    return a;
}

void PointPathAction::executeAction()
{
    bool success = executor->execute(shared_from_this());

    if(!success)
    {
        state = Action::State::FAILED;
    }
}

bool PointPathAction::triggerReplan()
{
    return executor->triggerReplan(shared_from_this());
}

void PointPathAction::monitor()
{
    executor->monitor(shared_from_this());
}

void PointPathAction::reset()
{
    currentPoint = 0;
    pointReachedTimes.clear();
    doInterruptPoint = false;
    state = Action::State::PLANNED;
}

void PointPathAction::cancel()
{
    executor->cancel(shared_from_this());
    state = Action::State::INTERRUPTED;
}

void PointPathAction::setCurrentPoint(const int point)
{
    currentPoint = point;
}

const int PointPathAction::getCurrentPoint() const
{
    return currentPoint;
}

void PointPathAction::setGoingUp(const bool goingUp)
{
    this->goingUp = goingUp;
}

const bool PointPathAction::getGoingUp()
{
    return goingUp;
}

void PointPathAction::setInterruptPoint(tf::Vector3 point)
{
    doInterruptPoint = true;
    interruptPoint = point;
}

void PointPathAction::disableInterruptPoint()
{
    doInterruptPoint = false;
}

tf::Vector3& PointPathAction::getInterruptPoint()
{
    return interruptPoint;
}

bool PointPathAction::getDoInterruptPoint()
{
    return doInterruptPoint;
}

void PointPathAction::addPointReachedTime(const ros::Time& time)
{
    pointReachedTimes.push_back(time);
}

const std::vector<ros::Time>& PointPathAction::getPointReachedTimes()
{
    return pointReachedTimes;
}