#include "planner_framework/PlannerData.h"

PlannerData::PlannerData(double time, VehiclePose pose, std::map<std::string, double> data) :
    time(time),
    pose(pose),
    data(data)
{}

PlannerData::PlannerData() :
    time(0),
    pose(VehiclePose(0,0,0)),
    data(std::map<std::string, double>())
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