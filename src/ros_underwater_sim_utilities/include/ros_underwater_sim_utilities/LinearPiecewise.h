#ifndef LINEAR_PIECEWISE_H
#define LINEAR_PIECEWISE_H

#include <vector>
#include <algorithm>

/**
* A linear piecewise function.
*/
class LinearPiecewise
{
public:
    /**
    * A point that defines a part of the linear piecewise function.
    */
    struct Point
    {
        double x;
        double y;

        bool operator< (const Point& other) const;
        bool operator== (const Point& other) const;
    };

    LinearPiecewise(std::vector<Point>& points);
    LinearPiecewise() {}
    ~LinearPiecewise() {}

    /**
    * Get y from x input.
    * @param x The input x value.
    * @return The Y value corresponding to the given X.
    */
    Point getY(double x);

    /**
    * Get all possible x from y inputs
    * @param y Y input
    * @return All possible X values that correspond to the given Y.
    */
    std::vector<Point> getX(double y);    


private:
    std::vector<Point> points;
};

#endif