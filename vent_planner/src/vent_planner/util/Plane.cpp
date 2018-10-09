#include <limits>
#include <iostream>
#include <string>

#include "ros/ros.h"

#include "vent_planner/util/Plane.h"

Plane::Plane(double a, double b, double c, double d) :
    a(a),
    b(b),
    c(c),
    d(d)
{}

Plane::Plane(tf::Vector3 normal, tf::Vector3 point) :
    a(normal.getX()),
    b(normal.getY()),
    c(normal.getZ())
{
    d = -(normal.getX() * point.getX() + 
          normal.getY() * point.getY() +
          normal.getZ() * point.getZ());
}

void Plane::reverse()
{
    a = -a;
    b = -b;
    c = -c;
    d = -d;
}

double Plane::getHeightGradientHeading()
{
    if(a == 0 && b == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    //reverse plane normal direction if z is pointing negative
    if(c < 0)
    {
        reverse();
    }

    tf::Vector3 normal(a, b, 0);
    tf::Vector3 north(0,1,0);
    tf::Vector3 cross = north.cross(normal);

    if(cross.getZ() >= 0)
    {
        return -north.angle(normal);
    }
    else
    {
        return north.angle(normal);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool Plane::equals(Plane& other)
{
    tf::Vector3 point(0, 0, getZ(0, 0));
    tf::Vector3 pointNormal(0, 0, other.getZ(0, 0));

    tf::Vector3 normal = getNormal();
    tf::Vector3 otherNormal = other.getNormal();

    normal.normalize();
    otherNormal.normalize();

    double parallelDot = normal.dot(otherNormal) / 
                         (normal.length() * otherNormal.length());

    double thisD = point.dot(normal);
    double otherD = pointNormal.dot(otherNormal);

    return 1 - fabs(parallelDot) < 0.000001 &&
           fabs(fabs(thisD) - fabs(otherD)) < 0.000001;
}

tf::Vector3 Plane::getNormal()
{
    return tf::Vector3(a, b, c);
}

double Plane::getZ(double x, double y)
{
    return (a * x + b * y + d) / -c;
}

Plane Plane::fitPlaneToPoints(std::vector<tf::Vector3>& points)
{
    //compute mean
    tf::Vector3 mean(0,0,0);

    for(tf::Vector3& point : points)
    {
        mean += point;
    }
    mean /= points.size();

    double xxSum = 0;
    double xySum = 0;
    double xhSum = 0;
    double yySum = 0;
    double yhSum = 0;

    for(tf::Vector3& point : points)
    {
        tf::Vector3 diff = point - mean;
        xxSum += diff.getX() * diff.getX();
        xySum += diff.getX() * diff.getY();
        xhSum += diff.getX() * diff.getZ();
        yySum += diff.getY() * diff.getY();
        yhSum += diff.getY() * diff.getZ();
    }

    double det = xxSum * yySum - xySum * xySum;
    if(det != 0)
    {
        double barA0 = (yySum * xhSum - xySum * yhSum) / det;
        double barA1 = (xxSum * yhSum - xySum * xhSum) / det;

        return Plane(barA0, barA1, -1, 
                    mean.getZ() - barA0 * mean.getX() - barA1 * mean.getY());
    }

    throw std::runtime_error("Invalid inputs to plane fitting.");
}