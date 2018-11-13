#ifndef CREATE_PATH_UTIL
#define CREATE_PATH_UTIL

#include <vector>

#include "planner_framework/VehiclePose.h"

namespace create_path_util
{
    std::vector<VehiclePose> makeSpiral(VehiclePose startLocation, double startDirection, double spacing, double size);

    std::vector<VehiclePose> makeLawnmower(const VehiclePose& startLocation,
                                           double alongTrackDirection,
                                           double acrossTrackDirection,
                                           double alongTrackSize,
                                           double acrossTrackSize,
                                           double spacing);

    std::vector<VehiclePose> makePolygon(const VehiclePose& center,
                                         const unsigned int sides,
                                         const double radius,
                                         const double initalPointHeading,
                                         const bool clockwise);
}

#endif