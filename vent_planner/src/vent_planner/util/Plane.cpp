#include <limits>
#include <iostream>
#include <string>

#include "vent_planner/util/Plane.h"

Plane::Plane(double a, double b, double c, double d) :
    a(a),
    b(b),
    c(c),
    d(d)
{}

Plane::Plane(Eigen::Vector3d normal, Eigen::Vector3d point) :
    a(normal[0]),
    b(normal[1]),
    c(normal[2])
{
    d = -(normal[0] * point[0] + 
          normal[1] * point[1] +
          normal[2] * point[2]);
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

    Eigen::Vector3d normal(a, b, 0);
    Eigen::Vector3d north(0,1,0);
    Eigen::Vector3d cross = normal.cross(north);

    if(cross[2] <= 0)
    {
        //angle between north and normal
        return -acos(north.dot(normal) / (north.norm() * normal.norm()));
    }
    else
    {
        //angle between north and normal
        return acos(north.dot(normal) / (north.norm() * normal.norm()));
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool Plane::equals(Plane& other)
{
    Eigen::Vector3d point(0, 0, getZ(0, 0));
    Eigen::Vector3d pointNormal(0, 0, other.getZ(0, 0));

    Eigen::Vector3d normal = getNormal();
    Eigen::Vector3d otherNormal = other.getNormal();

    normal.normalize();
    otherNormal.normalize();

    double parallelDot = normal.dot(otherNormal) / 
                         (normal.norm() * otherNormal.norm()); //.norm() is vector magnitude

    double thisD = point.dot(normal);
    double otherD = pointNormal.dot(otherNormal);

    return 1 - fabs(parallelDot) < 0.000001 &&
           fabs(fabs(thisD) - fabs(otherD)) < 0.000001;
}



Eigen::Vector3d Plane::getNormal()
{
    return Eigen::Vector3d(a, b, c);
}

double Plane::getZ(double x, double y)
{
    return (a * x + b * y + d) / -c;
}

Plane Plane::fitPlaneToPoints(std::vector<PlannerData>& points)
{
    //compute mean
    Eigen::Vector3d mean(0,0,0);

    for(PlannerData& data : points)
    {
        Eigen::Vector3d p(data.getPose().getX(),
                      data.getPose().getY(),
                      data.getData()["plume"]);
        mean += p;
    }
    mean /= points.size();

    double xxSum = 0;
    double xySum = 0;
    double xhSum = 0;
    double yySum = 0;
    double yhSum = 0;

    for(PlannerData& data : points)
    {
        Eigen::Vector3d p(data.getPose().getX(),
                      data.getPose().getY(),
                      data.getData()["plume"]);

        Eigen::Vector3d diff = p - mean;
        xxSum += diff[0] * diff[0];
        xySum += diff[0] * diff[1];
        xhSum += diff[0] * diff[2];
        yySum += diff[1] * diff[1];
        yhSum += diff[1] * diff[2];
    }

    double det = xxSum * yySum - xySum * xySum;
    if(det != 0)
    {
        double barA0 = (yySum * xhSum - xySum * yhSum) / det;
        double barA1 = (xxSum * yhSum - xySum * xhSum) / det;

        return Plane(barA0, barA1, -1, 
                    mean[2] - barA0 * mean[0] - barA1 * mean[1]);
    }

    throw std::runtime_error("Invalid inputs to plane fitting.");
}