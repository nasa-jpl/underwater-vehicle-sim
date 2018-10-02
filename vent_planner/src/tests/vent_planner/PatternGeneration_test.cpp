#include <gtest/gtest.h>

#include <math.h>

#include "tf/LinearMath/Vector3.h"

#include "vent_planner/CreatePathUtil.h"

TEST(PatternGeneration, Spiral)
{
    std::vector<tf::Vector3> expectedResult;

    expectedResult.emplace_back(100.0,200.0,-100.0);
    expectedResult.emplace_back(100.0,300.0,-100.0);
    expectedResult.emplace_back(0.0,300.0,-100.0);
    expectedResult.emplace_back(0.0,100.0,-100.0);
    expectedResult.emplace_back(200.0,100.0,-100.0);
    expectedResult.emplace_back(200.0,300.0,-100.0);

    tf::Vector3 startLocation(100, 200, -100); 

    double startDirection = M_PI / 2;
    double spacing = 100;
    double size = 210;

    std::vector<tf::Vector3> spiral = create_path_util::makeSpiral(startLocation, 
                                                              startDirection, 
                                                              spacing, 
                                                              size);

    ASSERT_EQ(expectedResult.size(), spiral.size());
    for(unsigned i = 0; i < spiral.size(); i++)
    {
        ASSERT_NEAR(expectedResult[i].getX(), spiral[i].getX(), 0.0000001);
        ASSERT_NEAR(expectedResult[i].getY(), spiral[i].getY(), 0.0000001);
        ASSERT_NEAR(expectedResult[i].getZ(), spiral[i].getZ(), 0.0000001);
    }
}

TEST(PatternGeneration, Lawnmower)
{
    std::vector<tf::Vector3> expectedResult;

    expectedResult.emplace_back(100.0,200.0,-100.0);
    expectedResult.emplace_back(100.0,300.0,-100.0);
    expectedResult.emplace_back(200.0,300.0,-100.0);
    expectedResult.emplace_back(200.0,200.0,-100.0);
    expectedResult.emplace_back(300.0,200.0,-100.0);
    expectedResult.emplace_back(300.0,300.0,-100.0);
    expectedResult.emplace_back(400.0,300.0,-100.0);
    expectedResult.emplace_back(400.0,200.0,-100.0);

    tf::Vector3 startLocation(100, 200, -100);

    double alongTrackDirection = M_PI / 2;
    double acrossTrackDirection = 0;
    double alongTrackSize = 100;
    double acrossTrackSize = 300;
    double spacing = 100;

    std::vector<tf::Vector3> lawnmower = create_path_util::makeLawnmower(startLocation,
                               alongTrackDirection,
                               acrossTrackDirection,
                               alongTrackSize,
                               acrossTrackSize,
                               spacing);

    ASSERT_EQ(expectedResult.size(), lawnmower.size());
    for(unsigned i = 0; i < lawnmower.size(); i++)
    {
        ASSERT_NEAR(expectedResult[i].getX(), lawnmower[i].getX(), 0.0000001);
        ASSERT_NEAR(expectedResult[i].getY(), lawnmower[i].getY(), 0.0000001);
        ASSERT_NEAR(expectedResult[i].getZ(), lawnmower[i].getZ(), 0.0000001);
    }

}

TEST(PatternGeneration, Polygon)
{
    std::vector<tf::Vector3> expectedResult1;

    expectedResult1.emplace_back(100.0, 250.0,-100.0);
    expectedResult1.emplace_back(143.3012701892, 175.0,-100.0);
    expectedResult1.emplace_back(56.6987298108, 175.0, -100.0);
    expectedResult1.emplace_back(100.0, 250.0, -100.0);

    std::vector<tf::Vector3> expectedResult2;

    expectedResult2.emplace_back(-29.2893218813, 170.710678119, -100.0);
    expectedResult2.emplace_back(-125.88190451, 196.592582629,-100.0);
    expectedResult2.emplace_back(-196.592582629, 125.88190451,-100.0);
    expectedResult2.emplace_back(-170.710678119, 29.2893218813,-100.0);
    expectedResult2.emplace_back(-74.1180954897, 3.40741737109,-100.0);
    expectedResult2.emplace_back(-3.40741737109, 74.1180954897,-100.0);
    expectedResult2.emplace_back(-29.2893218813, 170.710678119, -100.0);


    tf::Vector3 center1(100, 200, -100);
    unsigned int sides = 3;
    double radius = 50;
    double initalPointHeading = 0;
    bool clockwise = true;

    std::vector<tf::Vector3> result1 = create_path_util::makePolygon(center1,
                                                                     sides,
                                                                     radius,
                                                                     initalPointHeading,
                                                                     clockwise);

    tf::Vector3 center2(-100, 100, -100);
    sides = 6;
    radius = 100;
    initalPointHeading = M_PI / 4;
    clockwise = false;

    std::vector<tf::Vector3> result2 = create_path_util::makePolygon(center2,
                                                                     sides,
                                                                     radius,
                                                                     initalPointHeading,
                                                                     clockwise);


    ASSERT_EQ(expectedResult1.size(), result1.size());
    for(unsigned i = 0; i < result1.size(); i++)
    {
        ASSERT_NEAR(expectedResult1[i].getX(), result1[i].getX(), 0.0000001);
        ASSERT_NEAR(expectedResult1[i].getY(), result1[i].getY(), 0.0000001);
        ASSERT_NEAR(expectedResult1[i].getZ(), result1[i].getZ(), 0.0000001);
    }

    ASSERT_EQ(expectedResult2.size(), result2.size());
    for(unsigned i = 0; i < result2.size(); i++)
    {
        ASSERT_NEAR(expectedResult2[i].getX(), result2[i].getX(), 0.0000001);
        ASSERT_NEAR(expectedResult2[i].getY(), result2[i].getY(), 0.0000001);
        ASSERT_NEAR(expectedResult2[i].getZ(), result2[i].getZ(), 0.0000001);
    }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
