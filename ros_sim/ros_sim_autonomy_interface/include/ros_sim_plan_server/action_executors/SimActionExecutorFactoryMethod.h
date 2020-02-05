#ifndef CREATE_SIM_ACTION_EXECUTOR_H
#define CREATE_SIM_ACTION_EXECUTOR_H

#include <exception>
#include <memory>
#include <functional>

#include "underwater_autonomy/planner/ActionExecutor.h"

template <class T>
class SimActionExecutorFactoryMethod
{
public:

    static std::unique_ptr<T> create(ros::NodeHandle& nh, VehicleInfo& info)
    {
        return std::unique_ptr<T>(new T(nh, info));
    }
};

#endif