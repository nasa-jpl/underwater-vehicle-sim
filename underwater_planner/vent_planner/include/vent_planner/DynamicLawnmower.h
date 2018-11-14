#ifndef DYNAMIC_LAWNMOWER_H
#define DYNAMIC_LAWNMOWER_H

#include <vector>

#include "planner_framework/Action.h"
#include "planner_framework/PlannerData.h"
#include "vent_planner/actions/VentActionFactory.h"

class DynamicLawnmower
{
public:
    DynamicLawnmower(VehiclePose startLocation,
                     double alongTrackDirection,
                     double acrossTrackDirection,
                     double trackSpacing,
                     double targetHeight,
                     int minSectionsPerTrack,
                     double continueThreshold,
                     int trackSectionThreshold);

    ~DynamicLawnmower() {}

    void completeSection(std::vector<PlannerData>& data);

    bool isDone();
    VehiclePose getNextPoint();

private:
    bool processData(std::vector<PlannerData>& data);

private:
    //Parameters
    VehiclePose startLocation;
    double alongTrackDirection;
    double acrossTrackDirection;
    double trackSpacing;
    double targetHeight;
    int minSectionsPerTrack;
    double continueThreshold;
    int trackSectionThreshold;

    //State Variables
    int currentTrack;
    int lastTrack;
    int currentSection;
    int sectionsUnderThreshold;
    bool trackUnderThreshold;
    int sectionsCompletedInTrack;
    bool dynamicLawnmowerComplete;

    std::vector<double> sectionAverages;
};

#endif