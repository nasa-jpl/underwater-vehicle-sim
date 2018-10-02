#ifndef CREATE_PATH_UTIL
#define CREATE_PATH_UTIL

#include "tf/LinearMath/Vector3.h"
#include <vector>

namespace create_path_util
{
    std::vector<tf::Vector3> makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size);

    std::vector<tf::Vector3> makeLawnmower(const tf::Vector3& startLocation,
                                           double alongTrackDirection,
                                           double acrossTrackDirection,
                                           double alongTrackSize,
                                           double acrossTrackSize,
                                           double spacing);
}

#endif