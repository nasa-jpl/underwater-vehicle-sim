#ifndef LINEAR_PIECEWISE
#define LINEAR_PIECEWISE

#include <vector>
#include <algorithm>

class LinearPiecewise
{
public:

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
    * Get y from x input
    */
    Point getY(double x);

    /**
    * Get all possible x from y inputs
    */
    std::vector<Point> getX(double y);    


private:
    std::vector<Point> points;
};

#endif