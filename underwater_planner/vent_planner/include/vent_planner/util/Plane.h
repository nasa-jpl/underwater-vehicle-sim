#ifndef PLANE_H
#define PLANE_H

#include <vector>

#include "planner_framework/PlannerData.h"

#include <eigen3/Eigen/Dense>

class Plane
{
public:
    Plane(double a, double b, double c, double d);
    Plane(Eigen::Vector3d normal, Eigen::Vector3d point);
    ~Plane() {}

    double getHeightGradientHeading();

    bool equals(Plane& other);

    static Plane fitPlaneToPoints(std::vector<PlannerData>& points);

    void reverse();

    Eigen::Vector3d getNormal();
    double getZ(double x, double y);
private:

private:
    double a;
    double b;
    double c;
    double d;
};

#endif