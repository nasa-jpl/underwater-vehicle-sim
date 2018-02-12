#include <gtest/gtest.h>

#include <math.h>

#include "tf/LinearMath/Vector3.h"

#include "vent_planner/NestedSpiralVentPlanner.h"

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

    std::vector<tf::Vector3> spiral = NestedSpiralVentPlanner::makeSpiral(startLocation, 
                                                              startDirection, 
                                                              spacing, 
                                                              size);

    ASSERT_EQ(expectedResult.size(), spiral.size());
    for(unsigned i = 0; i < spiral.size(); i++)
    {
        ASSERT_NEAR(expectedResult[i].getX(), spiral[i].getX(), 0.0000000001);
        ASSERT_NEAR(expectedResult[i].getY(), spiral[i].getY(), 0.0000000001);
        ASSERT_NEAR(expectedResult[i].getZ(), spiral[i].getZ(), 0.0000000001);
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

    std::vector<tf::Vector3> lawnmower = NestedSpiralVentPlanner::makeLawnmower(startLocation,
                               alongTrackDirection,
                               acrossTrackDirection,
                               alongTrackSize,
                               acrossTrackSize,
                               spacing);

    ASSERT_EQ(expectedResult.size(), lawnmower.size());
    for(unsigned i = 0; i < lawnmower.size(); i++)
    {
        ASSERT_NEAR(expectedResult[i].getX(), lawnmower[i].getX(), 0.0000000001);
        ASSERT_NEAR(expectedResult[i].getY(), lawnmower[i].getY(), 0.0000000001);
        ASSERT_NEAR(expectedResult[i].getZ(), lawnmower[i].getZ(), 0.0000000001);
    }

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
