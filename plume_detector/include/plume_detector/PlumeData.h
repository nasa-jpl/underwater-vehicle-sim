#ifndef PLUME_DATA_H
#define PLUME_DATA_H

#include "ros/ros.h"

struct PlumeData
{
    PlumeData() :
        time(ros::Time(0)),
        x(0),
        y(0),
        h(0),
        val(0)
    {}

    PlumeData(ros::Time time, double x, double y, double h, double val) :
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