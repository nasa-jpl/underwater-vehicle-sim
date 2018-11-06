#ifndef PLAN_DATA_H
#define PLAN_DATA_H

#include <string>
#include <map>

#include "planner_framework/VehiclePose.h"

class PlannerData
{

public:
    PlannerData(double time, VehiclePose pose, std::map<std::string, double> data);
    ~PlannerData() {}

    double getTime() const;
    VehiclePose getPose() const;
    std::map<std::string, double> getData() const;
private:
    double time;
    VehiclePose pose;
    std::map<std::string, double> data;
};

#endif