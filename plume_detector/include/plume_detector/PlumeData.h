#ifndef PLUME_DATA_H
#define PLUME_DATA_H

#include "ros/ros.h"

struct PlumeData
{
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