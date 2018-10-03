#ifndef PLANE_H
#define PLANE_H

#include <vector>

#include "tf/LinearMath/Vector3.h"

class Plane
{
public:
    Plane(double a, double b, double c, double d);
    Plane(tf::Vector3 normal, tf::Vector3 point);
    ~Plane() {}

    double getHeightGradientHeading();

    bool equals(Plane& other);

    static Plane fitPlaneToPoints(std::vector<tf::Vector3>& points);

    void reverse();

    tf::Vector3 getNormal();
    double getZ(double x, double y);
private:

private:
    double a;
    double b;
    double c;
    double d;
};

#endif