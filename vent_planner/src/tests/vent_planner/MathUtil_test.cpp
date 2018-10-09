#include <gtest/gtest.h>

#include <math.h>
#include <cmath>

#include "tf/LinearMath/Vector3.h"

#include "vent_planner/util/MathUtil.h"

TEST(MathUtil, LinearLeastSquares)
{
    std::vector<double> x1;
    std::vector<double> y1;

    x1.push_back(1.2);
    y1.push_back(0.1);

    x1.push_back(1.9);
    y1.push_back(0.7);

    x1.push_back(2.3);
    y1.push_back(2.1);

    x1.push_back(3.3);
    y1.push_back(4.2);

    x1.push_back(2.5);
    y1.push_back(4.1);

    x1.push_back(6.05);
    y1.push_back(7.1);

    double slope1;
    double yIntercept1;

    math_util::linearLeastSquares(x1, y1, slope1, yIntercept1);

    ASSERT_NEAR(slope1, 1.430937797, 0.000001);
    ASSERT_NEAR(yIntercept1, -1.063946165, 0.000001);

    std::vector<double> x2;
    std::vector<double> y2;

    x2.push_back(1.2);
    y2.push_back(7.1);

    x2.push_back(1.9);
    y2.push_back(4.1);
    
    x2.push_back(2.3);
    y2.push_back(4.2);
    
    x2.push_back(3.3);
    y2.push_back(2.1);

    x2.push_back(2.5);
    y2.push_back(0.7);

    x2.push_back(6.05);
    y2.push_back(0.1);
    

    double slope2;
    double yIntercept2;

    math_util::linearLeastSquares(x2, y2, slope2, yIntercept2);

    ASSERT_NEAR(slope2, -1.197998447, 0.000001);
    ASSERT_NEAR(yIntercept2, 6.494245535, 0.000001);


    std::vector<double> x3;
    std::vector<double> y3;

    double slope3;
    double yIntercept3;

    math_util::linearLeastSquares(x3, y3, slope3, yIntercept3);

    ASSERT_TRUE(std::isnan(slope3));
    ASSERT_TRUE(std::isnan(yIntercept3));

    std::vector<double> x4;
    std::vector<double> y4;

    x4.push_back(1);
    y4.push_back(7.1);

    x4.push_back(1);
    y4.push_back(4.1);
    

    double slope4;
    double yIntercept4;

    math_util::linearLeastSquares(x4, y4, slope4, yIntercept4);

    ASSERT_TRUE(std::isnan(slope4));
    ASSERT_TRUE(std::isnan(yIntercept4));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
