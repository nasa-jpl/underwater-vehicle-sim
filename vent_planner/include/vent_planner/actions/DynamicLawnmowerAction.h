#ifndef DYNAMIC_LAWNMOWER_ACTION_H
#define DYNAMIC_LAWNMOWER_ACTION_H

#include <vector>
#include <memory>
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Action.h"
#include "planner_framework/ActionExecutor.h"

class DynamicLawnmowerAction : public Action, public std::enable_shared_from_this<DynamicLawnmowerAction>
{
   

public:

DynamicLawnmowerAction(std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> executor,
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
                       const int trackSectionThreshold);

DynamicLawnmowerAction(const DynamicLawnmowerAction& action);

~DynamicLawnmowerAction() {}

std::shared_ptr<Action> clone() const override;

/**
*Executes the action using the provided executor
*/
void executeAction();

/**
* Allows the action to trigger a replan
*/
bool triggerReplan();

/**
* Monitors the state of the action and updates it as needed
*/
void monitor();

/**
*Resets this action to a state as if it has not been executed.
*/
void reset();

void cancel();

int getCurrentTrack();
int getCurrentSection();

void setCurrentTrack(int currentTrack);
void setCurrentSection(int currentSection);

double getTargetHorizontalVelocity() const;
double getTargetRotationalVelocity() const;
double getTargetSlope() const;
tf::Vector3 getStartLocation() const;
double getAlongTrackDirection() const;
double getAcrossTrackDirection() const;
double getTrackSpacing() const;
double getTargetHeight() const;
int getMinSectionsPerTrack() const;
double getContinueThreshold() const;
int getTrackSectionThreshold() const;


private:
std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> executor;

//Parameters
const double targetHorizontalVelocity;
const double targetRotationalVelocity;
const double targetSlope;
const tf::Vector3 startLocation;
const double alongTrackDirection;
const double acrossTrackDirection;
const double trackSpacing;
const double targetHeight;
const int minSectionsPerTrack;
const double continueThreshold;
const int trackSectionThreshold;

//Track action progress
int currentTrack;
int currentSection;

};

#endif