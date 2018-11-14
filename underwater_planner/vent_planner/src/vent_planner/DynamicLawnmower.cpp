#include "vent_planner/DynamicLawnmower.h"

#include <cmath>

DynamicLawnmower::DynamicLawnmower(VehiclePose startLocation,
                                   double alongTrackDirection,
                                   double acrossTrackDirection,
                                   double trackSpacing,
                                   double targetHeight,
                                   int minSectionsPerTrack,
                                   double continueThreshold,
                                   int trackSectionThreshold) :
    startLocation(startLocation),
    alongTrackDirection(alongTrackDirection),
    acrossTrackDirection(acrossTrackDirection),
    trackSpacing(trackSpacing),
    targetHeight(targetHeight),
    minSectionsPerTrack(minSectionsPerTrack),
    continueThreshold(continueThreshold),
    trackSectionThreshold(trackSectionThreshold),
    currentTrack(0),
    lastTrack(-1),
    currentSection(0),
    sectionsUnderThreshold(0),
    trackUnderThreshold(true),
    sectionsCompletedInTrack(0),
    dynamicLawnmowerComplete(false)

{}

void DynamicLawnmower::completeSection(std::vector<PlannerData>& data)
{    
    if(lastTrack == currentTrack)
    {
        bool overThresh = processData(data);
        if(!overThresh)
        {
            sectionsUnderThreshold++;
        }
        else
        {
            trackUnderThreshold = false;
            sectionsUnderThreshold = 0;
        }
    }
    lastTrack = currentTrack;

    bool nextTrack = true;

    if(sectionsCompletedInTrack >= minSectionsPerTrack &&
        sectionsUnderThreshold >= trackSectionThreshold)
    {
        //Determines if the average for each section is less than the last.
        //This prevents the vehicle from turning if heading towards more plume
        for(unsigned int i = sectionAverages.size() - trackSectionThreshold; i < sectionAverages.size() - 1; i++)
        {
            if(sectionAverages[i] < sectionAverages[i + 1])
            {
                nextTrack = false;
                break;
            }
        }
    }
    else
    {
        nextTrack = false;
    }

    //Update current section and current track accordingly
    if(nextTrack || (currentTrack % 2 == 1 && currentSection == 0))
    {
        currentTrack++;
        if(!trackUnderThreshold)
        {
            //reset the consecutive sections under the threshold
            sectionsUnderThreshold = 0;
            trackUnderThreshold = true;

            sectionsCompletedInTrack = 0;
            sectionAverages.clear();
        }
        else
        {
            dynamicLawnmowerComplete = true;
        }
    }
    else
    {
        if(currentTrack % 2 == 0)
        {
            currentSection++;
        }
        else
        {
            currentSection--;
        }

        sectionsCompletedInTrack++;
    }
}

bool DynamicLawnmower::isDone()
{
    return dynamicLawnmowerComplete;
}

VehiclePose DynamicLawnmower::getNextPoint()
{
    VehiclePose point;

    //Calculate across track location
    point.setX(startLocation.getX() + cos(acrossTrackDirection) * trackSpacing * currentTrack 
                                    + cos(alongTrackDirection) * trackSpacing * currentSection);

    point.setY(startLocation.getY() + sin(acrossTrackDirection) * trackSpacing * currentTrack
                                    + sin(alongTrackDirection) * trackSpacing * currentSection);

    point.setZ(startLocation.getZ());

    return point;
}

bool DynamicLawnmower::processData(std::vector<PlannerData>& data)
{
    bool overThresh = false;
    double averageVal = 0;
    for(unsigned int i = 0; i < data.size(); i++)
    {
        if(data[i].getData()["plume"] >= continueThreshold)
        {
            overThresh = true;
        }
        averageVal += data[i].getData()["plume"];
    }

    averageVal /= data.size();
    sectionAverages.push_back(averageVal);

    return overThresh;
}