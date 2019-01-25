#include <gtest/gtest.h>
#include <cmath>

#include "vehicles/LinearPiecewise.h"

TEST(LinearPiecewise, getY)
{
    //Points added out of order to test sorting in constructor
    std::vector<LinearPiecewise::Point> points;
    points.push_back({-5, -4});
    points.push_back({10, 3});
    points.push_back({5, 2});
    points.push_back({0, 0});
    points.push_back({-10, -5});

    LinearPiecewise func(points);

    std::vector<LinearPiecewise::Point> targetPoints;
    targetPoints.push_back({-10.1, std::numeric_limits<double>::quiet_NaN()});
    targetPoints.push_back({-10, -5});
    targetPoints.push_back({-9, -4.8});
    targetPoints.push_back({5, 2});
    targetPoints.push_back({1, 0.4});
    targetPoints.push_back({10, 3});
    targetPoints.push_back({10.1, std::numeric_limits<double>::quiet_NaN()});

    //Check valid values
    for(unsigned int i = 0; i < targetPoints.size(); i++)
    {
        LinearPiecewise::Point returnedPoint = func.getY(targetPoints[i].x);
        EXPECT_TRUE(targetPoints[i] == returnedPoint);
    }
}

TEST(LinearPiecewise, getX)
{
    //Points added out of order to test sorting in constructor
    std::vector<LinearPiecewise::Point> points;
    points.push_back({-10, 5});
    points.push_back({-5, 4});
    points.push_back({0, 0});
    points.push_back({5, 2});
    points.push_back({10, 3});
    
    std::vector<std::vector<LinearPiecewise::Point>> targetPoints;
    std::vector<LinearPiecewise::Point> vec0;
    vec0.push_back({-10, 5});

    std::vector<LinearPiecewise::Point> vec1;
    vec1.push_back({-3.75, 3});
    vec1.push_back({10, 3});

    std::vector<LinearPiecewise::Point> vec2;
    vec2.push_back({-1.25, 1});
    vec2.push_back({2.5, 1});

    targetPoints.push_back(vec0);
    targetPoints.push_back(vec1);
    targetPoints.push_back(vec2);

    LinearPiecewise func(points);
    for(unsigned int i = 0; i < targetPoints.size(); i++)
    {
        std::vector<LinearPiecewise::Point> returnedPoints = func.getX(targetPoints[i][0].y);
        EXPECT_TRUE(targetPoints[i] == returnedPoints);
    }

    std::vector<LinearPiecewise::Point> notInRange0 = func.getX(5.1);
    std::vector<LinearPiecewise::Point> notInRange1 = func.getX(-0.1);

    EXPECT_EQ(0, notInRange0.size());
    EXPECT_EQ(0, notInRange1.size());
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
