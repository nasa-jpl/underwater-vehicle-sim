#ifndef MATH_UTIL
#define MATH_UTIL

#include "tf/LinearMath/Vector3.h"
#include <vector>

namespace math_util
{
    void linearLeastSquares(const std::vector<double>& x, std::vector<double>& y, double &slope, double &yIntercept);

    double xyDistance(tf::Vector3 p1, tf::Vector3 p2);
}

#endif