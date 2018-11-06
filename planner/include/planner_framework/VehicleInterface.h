#ifndef VEHICLE_INTERFACE_H
#define VEHICLE_INTERFACE_H

#include "planner_framework/VehiclePose.h"
#include "planner_framework/PlannerData.h"

class VehicleInterface
{
public:
    VehicleInterface() {}
    virtual ~VehicleInterface() {}

    virtual void getData()=0;
    virtual void registerDataCallback(std::function<void(const PlannerData&)> cb)=0;
    virtual VehiclePose getPosition()=0;
private:

};

#endif