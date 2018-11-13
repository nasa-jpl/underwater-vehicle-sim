#ifndef MATH_UTIL
#define MATH_UTIL

#include <vector>

#include "planner_framework/VehiclePose.h"

namespace math_util
{
    void linearLeastSquares(const std::vector<double>& x, std::vector<double>& y, double &slope, double &yIntercept);

    double xyDistance(VehiclePose p1, VehiclePose p2);
}

#endif