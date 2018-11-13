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

void VehiclePose::setX(double x)
{
    this->x = x;
}

void VehiclePose::setY(double y)
{
    this->y = y;
}

void VehiclePose::setZ(double z)
{
    this->z = z;
}

double VehiclePose::getX() const
{
    return x;
}

double VehiclePose::getY() const
{
    return y;
}

double VehiclePose::getZ() const
{
    return z;
}