#ifndef PLUME_DATA_H
#define PLUME_DATA_H

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