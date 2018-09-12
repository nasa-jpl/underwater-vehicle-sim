#include <memory>

#include "ros/ros.h"

#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "planner_framework/ActionExecutor.h"


DynamicLawnmowerAction::DynamicLawnmowerAction(std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> executor,
                                               const double targetHorizontalVelocity, 
                                               const double targetRotationalVelocity,
                                               const double targetSlope,
                                               const tf::Vector3& startLocation,
                                               const double alongTrackDirection,
                                               const double acrossTrackDirection,
                                               const double trackSpacing,
                                               const double targetHeight,
                                               const int minSectionsPerTrack,
                                               const double continueThreshold,
                                               const int trackSectionThreshold) :
    executor(std::move(executor)),
    targetHorizontalVelocity(targetHorizontalVelocity),
    targetRotationalVelocity(targetRotationalVelocity),
    targetSlope(targetSlope),
    startLocation(startLocation),
    alongTrackDirection(alongTrackDirection),
    acrossTrackDirection(acrossTrackDirection),
    trackSpacing(trackSpacing),
    minSectionsPerTrack(minSectionsPerTrack),
    continueThreshold(continueThreshold),
    trackSectionThreshold(trackSectionThreshold),
    targetHeight(targetHeight),
    currentTrack(0),
    currentSection(0)
{}

DynamicLawnmowerAction::DynamicLawnmowerAction(const DynamicLawnmowerAction& action) :
    Action(action),
    executor(action.executor->clone()),
    targetHorizontalVelocity(action.targetHorizontalVelocity),
    targetRotationalVelocity(action.targetRotationalVelocity),
    targetSlope(action.targetSlope),
    startLocation(action.startLocation),
    alongTrackDirection(action.alongTrackDirection),
    acrossTrackDirection(action.acrossTrackDirection),
    trackSpacing(action.trackSpacing),
    minSectionsPerTrack(action.minSectionsPerTrack),
    continueThreshold(action.continueThreshold),
    trackSectionThreshold(action.trackSectionThreshold),
    targetHeight(action.targetHeight),
    currentTrack(action.currentTrack),
    currentSection(action.currentSection)
{}

std::shared_ptr<Action> DynamicLawnmowerAction::clone() const
{
    std::shared_ptr<Action> a(new DynamicLawnmowerAction(*this));
    return a;
}

void DynamicLawnmowerAction::executeAction()
{
    ROS_INFO("Planner: Dynamic Lawnmower Action, call executor.execute()");
    bool success = executor->execute(shared_from_this());

    if(!success)
    {
        state = Action::State::FAILED;
    }
}

bool DynamicLawnmowerAction::triggerReplan()
{
    return executor->triggerReplan(shared_from_this());
}

void DynamicLawnmowerAction::monitor()
{
    executor->monitor(shared_from_this());
}

void DynamicLawnmowerAction::reset()
{
    currentTrack = 0;
    currentSection = 0;
    state = Action::State::PLANNED;
}

void DynamicLawnmowerAction::cancel()
{
    executor->cancel(shared_from_this());
    state = Action::State::INTERRUPTED;
}

int DynamicLawnmowerAction::getCurrentTrack()
{
    return currentTrack;
}

int DynamicLawnmowerAction::getCurrentSection()
{
    return currentSection;
}

void DynamicLawnmowerAction::setCurrentTrack(int currentTrack)
{
    this->currentTrack = currentTrack;
}

void DynamicLawnmowerAction::setCurrentSection(int currentSection)
{
    this->currentSection = currentSection;
}