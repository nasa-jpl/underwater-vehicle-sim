#include <limits>
#include <cmath>

#include "vent_planner/util/MathUtil.h"

void math_util::linearLeastSquares(const std::vector<double>& x, std::vector<double>& y, double &slope, double &yIntercept)
{
    if(x.size() != y.size() ||
       x.size() <= 1)
    {
        slope = std::numeric_limits<double>::quiet_NaN();
        yIntercept = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    double xBar = 0.0;
    double yBar = 0.0;

    for(unsigned int i = 0; i < x.size(); i++)
    {
        xBar += x[i];
        yBar += y[i];
    }
    xBar /= x.size();
    yBar /= y.size();

    double xy = 0.0;
    double xx = 0.0;

    for(unsigned int i = 0; i < x.size(); i++)
    {

        xy += (x[i] - xBar) * (y[i] - yBar);
        xx += (x[i] - xBar) * (x[i] - xBar);
    }

    if(xx > 0)
    {
        slope = xy / xx;
        yIntercept = yBar - slope * xBar;
    }
    else
    {
        slope = std::numeric_limits<double>::quiet_NaN();
        yIntercept = std::numeric_limits<double>::quiet_NaN();
    }
    
}

double math_util::xyDistance(VehiclePose p1, VehiclePose p2)
{
    return sqrt(((p1.getX() - p2.getX()) * (p1.getX() - p2.getX())) + 
                ((p1.getY() - p2.getY()) * (p1.getY() - p2.getY())));
}