#include <memory>

#include "vent_planner/actions/PointPathAction.h"
#include "planner_framework/ActionExecutor.h"

PointPathAction::PointPathAction(std::unique_ptr<ActionExecutor<PointPathAction>> executor,
                                 const double targetHorizontalVelocity,
                                 const double targetRotationalVelocity,
                                 const double targetSlope,
                                 const bool yoyo,
                                 const double upperDepth,
                                 const double lowerDepth,
                                 const std::vector<VehiclePose>& points,
                                 const ReplanType replanType,
                                 const double periodicReplanValue) :
    executor(std::move(executor)),
    targetHorizontalVelocity(targetHorizontalVelocity),
    targetRotationalVelocity(targetRotationalVelocity),
    targetSlope(targetSlope),
    yoyo(yoyo),
    upperDepth(upperDepth),
    lowerDepth(lowerDepth),
    points(points),
    replanType(replanType),
    periodicReplanValue(periodicReplanValue),
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
    periodicReplanValue(action.periodicReplanValue),
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

void PointPathAction::setInterruptPoint(VehiclePose point)
{
    doInterruptPoint = true;
    interruptPoint = point;
}

void PointPathAction::disableInterruptPoint()
{
    doInterruptPoint = false;
}

VehiclePose& PointPathAction::getInterruptPoint()
{
    return interruptPoint;
}

bool PointPathAction::getDoInterruptPoint()
{
    return doInterruptPoint;
}

void PointPathAction::addPointReachedTime(const double time)
{
    pointReachedTimes.push_back(time);
}

const std::vector<double>& PointPathAction::getPointReachedTimes()
{
    return pointReachedTimes;
}

double PointPathAction::getTargetHorizontalVelocity() const
{
    return targetHorizontalVelocity;
}

double PointPathAction::getTargetRotationalVelocity() const
{
    return targetRotationalVelocity;
}

std::vector<VehiclePose> PointPathAction::getPoints() const
{
    return points;
}

bool PointPathAction::getYoyo() const
{
    return yoyo;
}

double PointPathAction::getTargetSlope() const
{
    return targetSlope;
}

double PointPathAction::getUpperDepth() const
{
    return upperDepth;
}

double PointPathAction::getLowerDepth() const
{
    return lowerDepth;
}

PointPathAction::ReplanType PointPathAction::getReplanType() const
{
    return replanType;
}

double PointPathAction::getPeriodicReplanValue() const
{
    return periodicReplanValue;
}