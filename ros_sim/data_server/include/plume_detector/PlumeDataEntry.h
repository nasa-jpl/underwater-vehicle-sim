#ifndef PLUME_DATA_ENTRY_H
#define PLUME_DATA_ENTRY_H

#include "ros/ros.h"

struct PlumeDataEntry
{
    PlumeDataEntry() :
        time(ros::Time(0)),
        x(0),
        y(0),
        h(0),
        val(0)
    {}

    PlumeDataEntry(ros::Time time, double x, double y, double h, double val) :
        time(time),
        x(x),
        y(y),
        h(h),
        val(val) 
    {}

    ros::Time time;
    double x;
    double y;
    double h;
    double val;
};

#endif