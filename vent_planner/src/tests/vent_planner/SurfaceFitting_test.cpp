#include <gtest/gtest.h>

#include <cmath>

#include "tf/LinearMath/Vector3.h"
#include "vent_planner/util/Plane.h"

TEST(SurfaceFitting, PlaneFit)
{
    //TEST 1
    std::vector<tf::Vector3> pointsTest1;
    pointsTest1.emplace_back(0, 0, 0);
    pointsTest1.emplace_back(10, 10, 0);
    pointsTest1.emplace_back(10, -10, 0);

    Plane fitPlane1 = Plane::fitPlaneToPoints(pointsTest1);

    tf::Vector3 normalResultTest1(0,0,-1);
    tf::Vector3 pointResultTest1(0,0,0);
    Plane resultPlane1(normalResultTest1, pointResultTest1);

    ASSERT_TRUE(fitPlane1.equals(resultPlane1));
    ASSERT_TRUE(std::isnan(fitPlane1.getHeightGradientHeading()));

    //TEST 2
    std::vector<tf::Vector3> pointsTest2;
    pointsTest2.emplace_back(0, 0, 50);
    pointsTest2.emplace_back(10, 10, 50);
    pointsTest2.emplace_back(10, -10, 50);

    Plane fitPlane2 = Plane::fitPlaneToPoints(pointsTest2);

    tf::Vector3 normalResultTest2(0,0,-1);
    tf::Vector3 pointResultTest2(0,0,50);
    Plane resultPlane2(normalResultTest2, pointResultTest2);

    ASSERT_TRUE(fitPlane2.equals(resultPlane2));
    ASSERT_TRUE(std::isnan(fitPlane2.getHeightGradientHeading()));

    //TEST 3
    std::vector<tf::Vector3> pointsTest3;
    pointsTest3.emplace_back(0, 0, 60);
    pointsTest3.emplace_back(5, 0, 55);
    pointsTest3.emplace_back(10, 10, 50);
    pointsTest3.emplace_back(10, -10, 50);

    Plane fitPlane3 = Plane::fitPlaneToPoints(pointsTest3);

    tf::Vector3 normalResultTest3(1,0,1);
    tf::Vector3 pointResultTest3(0,0,60);
    Plane resultPlane3(normalResultTest3, pointResultTest3);

    ASSERT_TRUE(fitPlane3.equals(resultPlane3));
    ASSERT_NEAR(M_PI / 2, fitPlane3.getHeightGradientHeading(), 0.00001);

    //TEST 4
    std::vector<tf::Vector3> pointsTest4;
    pointsTest4.emplace_back(0, 0, 0);
    pointsTest4.emplace_back(10, 10, 0);
    pointsTest4.emplace_back(10, -10, 0);
    pointsTest4.emplace_back(0, 0, 0);
    pointsTest4.emplace_back(10, 10, 0);
    pointsTest4.emplace_back(10, -10, 0);
    pointsTest4.emplace_back(0, 0, 50);
    pointsTest4.emplace_back(10, 10, 50);
    pointsTest4.emplace_back(10, -10, 50);

    Plane fitPlane4 = Plane::fitPlaneToPoints(pointsTest4);

    tf::Vector3 normalResultTest4(0,0,-1);
    tf::Vector3 pointResultTest4(0,0,16.6666666667);
    Plane resultPlane4(normalResultTest4, pointResultTest4);

    ASSERT_TRUE(fitPlane4.equals(resultPlane4));
    ASSERT_TRUE(std::isnan(fitPlane4.getHeightGradientHeading()));

    //TEST 5
    std::vector<tf::Vector3> pointsTest5;
    pointsTest5.emplace_back(0, 0, 0);
    pointsTest5.emplace_back(10, 0, 0);
    pointsTest5.emplace_back(-10, 0, 0);

    double a0Test5 = 0;
    double a1Test5 = 0;
    double bTest5 = 0;

    try {
        Plane::fitPlaneToPoints(pointsTest5);
        FAIL() << "Expected std::runtime_error";
    }
    catch(std::runtime_error const & err) {
        EXPECT_EQ(err.what(),std::string("Invalid inputs to plane fitting."));
    }
    catch(...) {
        FAIL() << "Expected std::runtime_error";
    }

    //TEST 6
    std::vector<tf::Vector3> pointsTest6;
    pointsTest6.emplace_back(0, 0, 40);
    pointsTest6.emplace_back(5, 0, 45);
    pointsTest6.emplace_back(10, 10, 50);
    pointsTest6.emplace_back(10, -10, 50);

    Plane fitPlane6 = Plane::fitPlaneToPoints(pointsTest6);

    tf::Vector3 normalResultTest6(-1,0,1);
    tf::Vector3 pointResultTest6(0,0,40);
    Plane resultPlane6(normalResultTest6, pointResultTest6);

    ASSERT_TRUE(fitPlane6.equals(resultPlane6));
    ASSERT_NEAR(-M_PI / 2, fitPlane6.getHeightGradientHeading(), 0.00001);

    //TEST 7
    std::vector<tf::Vector3> pointsTest7;
    pointsTest7.emplace_back(0, 0, 40);
    pointsTest7.emplace_back(10, 10, 50);
    pointsTest7.emplace_back(-10, 10, 50);

    Plane fitPlane7 = Plane::fitPlaneToPoints(pointsTest7);

    tf::Vector3 normalResultTest7(0,-1,1);
    tf::Vector3 pointResultTest7(0,0,40);
    Plane resultPlane7(normalResultTest7, pointResultTest7);

    ASSERT_TRUE(fitPlane7.equals(resultPlane7));
    ASSERT_NEAR(-M_PI, fitPlane7.getHeightGradientHeading(), 0.00001);

    //TEST 8
    std::vector<tf::Vector3> pointsTest8;
    pointsTest8.emplace_back(0, 0, 60);
    pointsTest8.emplace_back(10, 10, 50);
    pointsTest8.emplace_back(-10, 10, 50);

    Plane fitPlane8 = Plane::fitPlaneToPoints(pointsTest8);

    tf::Vector3 normalResultTest8(0,1,1);
    tf::Vector3 pointResultTest8(0,0,60);
    Plane resultPlane8(normalResultTest8, pointResultTest8);

    ASSERT_TRUE(fitPlane8.equals(resultPlane8));
    ASSERT_NEAR(0, fitPlane8.getHeightGradientHeading(), 0.00001);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
