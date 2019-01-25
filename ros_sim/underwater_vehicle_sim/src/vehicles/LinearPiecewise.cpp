#include "vehicles/LinearPiecewise.h"

#include <cmath>
#include <limits>

bool LinearPiecewise::Point::operator< (const Point& other) const
{
    return x < other.x;
}

bool LinearPiecewise::Point::operator== (const Point& other) const
{
    return (std::fabs(x - other.x) < 0.0000001 || (std::isnan(x) && std::isnan(other.x))) &&
           (std::fabs(y - other.y) < 0.0000001 || (std::isnan(y) && std::isnan(other.y)));
}

LinearPiecewise::LinearPiecewise(std::vector<Point>& points) :
    points(points)
{
    std::sort(this->points.begin(), this->points.end());
}

LinearPiecewise::Point LinearPiecewise::getY(double x)
{
    std::vector<LinearPiecewise::Point>::iterator aboveX;
    std::vector<LinearPiecewise::Point>::iterator belowX;
    aboveX = std::upper_bound(points.begin(), points.end(), LinearPiecewise::Point{x, 0});
    if(aboveX == points.begin())
    {
        //Specifically check the lower boundary of the piecewise function 
        //to avoid any issues with floating point error
        if(x > aboveX->x - 0.000001)
        {
            return {x, aboveX->y}; 
        }
        else
        {
            return {x, std::numeric_limits<double>::quiet_NaN()};
        }
        return {x, std::numeric_limits<double>::quiet_NaN()};
    }

    if(aboveX == points.end())
    {
        //Specifically check the upper boundary of the piecewise function 
        //to avoid any issues with floating point error
        if(x < (aboveX - 1)->x + 0.000001)
        {
            return {x, (aboveX - 1)->y}; 
        }
        else
        {
            return {x, std::numeric_limits<double>::quiet_NaN()};
        }
    }

    belowX = aboveX - 1;

    double percentLower = (aboveX->x - x) / (aboveX->x - belowX->x);

    double yVal = belowX->y * percentLower + aboveX->y * (1 - percentLower);

    return {x, yVal};
}

std::vector<LinearPiecewise::Point> LinearPiecewise::getX(double y)
{
    std::vector<LinearPiecewise::Point> validPoints;

    for(unsigned int i = 1; i < points.size(); i++)
    {
        if((points[i - 1].y <= y && points[i].y > y) ||
           (points[i - 1].y >= y && points[i].y < y))
        {
            double percentLower = (points[i].y - y) / (points[i].y - points[i - 1].y);
            double xVal = points[i - 1].x * percentLower + points[i].x * (1 - percentLower);
            validPoints.push_back({xVal, y});
        }
        else if(i == 1 && fabs(points[i - 1].y - y) <= 0.000001)
        {
            //Check lower boundary to avoid floating point errors
            validPoints.push_back({points[i - 1].x, y});
        }
        else if(i == points.size() - 1 && fabs(points[i].y - y) <= 0.000001)
        {
            //Check upper boundary to avoid floating point errors
            validPoints.push_back({points[i].x, y});
        }
    }

    return validPoints;
}