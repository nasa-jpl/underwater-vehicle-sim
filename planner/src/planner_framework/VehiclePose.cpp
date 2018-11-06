#include "planner_framework/VehiclePose.h"

VehiclePose::VehiclePose() :
    x(0),
    y(0),
    z(0)
{}

VehiclePose::VehiclePose(double x, double y, double z) :
    x(x),
    y(y),
    z(z)
{}

double VehiclePose::getX()
{
    return x;
}

double VehiclePose::getY()
{
    return y;
}

double VehiclePose::getZ()
{
    return z;
}