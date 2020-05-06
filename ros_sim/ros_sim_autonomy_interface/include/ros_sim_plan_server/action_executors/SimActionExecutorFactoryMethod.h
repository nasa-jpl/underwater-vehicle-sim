#ifndef CREATE_SIM_ACTION_EXECUTOR_H
#define CREATE_SIM_ACTION_EXECUTOR_H

#include <exception>
#include <memory>
#include <functional>

#include "underwater_autonomy/planner/ActionExecutor.h"

template <class T1, class T2>
class SimActionExecutorFactoryMethod
{
public:

    static std::unique_ptr<T1> create(T2& action, ros::NodeHandle& nh, VehicleInfo& info)
    {
        return std::unique_ptr<T1>(new T1(action, nh, info));
    }
};

#endif