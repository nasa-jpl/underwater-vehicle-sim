#include "planner_framework/PlannerData.h"

PlannerData::PlannerData(double time, VehiclePose pose, std::map<std::string, double> data) :
    time(time),
    pose(pose),
    data(data)
{}

double PlannerData::getTime() const
{
    return time;
}

VehiclePose PlannerData::getPose() const
{
    return pose;
}

std::map<std::string, double> PlannerData::getData() const
{
    return data;
}