#include <gtest/gtest.h>

#include <cmath>

#include "tf/LinearMath/Vector3.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"

void planesEqual(tf::Vector3& normal1, tf::Vector3& point1,
                 tf::Vector3& normal2, tf::Vector3& point2)
{
    double parallelDot = normal1.dot(normal2) / (normal1.length() * normal2.length());

    normal1.normalize();
    normal2.normalize();

    double d1 = point1.dot(normal1);
    double d2 = point2.dot(normal2);

    ASSERT_NEAR(fabs(parallelDot), 1, 0.000000001);
    ASSERT_NEAR(fabs(d1), fabs(d2), 0.000000001);

}

TEST(SurfaceFitting, PlaneFit)
{
    //TEST 1
    std::vector<tf::Vector3> pointsTest1;
    pointsTest1.emplace_back(0, 0, 0);
    pointsTest1.emplace_back(10, 10, 0);
    pointsTest1.emplace_back(10, -10, 0);

    double a0Test1 = 0;
    double a1Test1 = 0;
    double bTest1 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest1, a0Test1, a1Test1, bTest1));

    tf::Vector3 normal1Test1(a0Test1, a1Test1, -1);
    tf::Vector3 normal2Test1(0,0,-1);

    tf::Vector3 point1Test1(0,0, bTest1);
    tf::Vector3 point2Test1(0,0,0);

    planesEqual(normal1Test1, point1Test1,
                normal2Test1, point2Test1);
    ASSERT_TRUE(std::isnan(SurfaceGradientVentPlanner::planeGradientHeading(a0Test1, a1Test1)));

    //TEST 2
    std::vector<tf::Vector3> pointsTest2;
    pointsTest2.emplace_back(0, 0, 50);
    pointsTest2.emplace_back(10, 10, 50);
    pointsTest2.emplace_back(10, -10, 50);

    double a0Test2 = 0;
    double a1Test2 = 0;
    double bTest2 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest2, a0Test2, a1Test2, bTest2));

    tf::Vector3 normal1Test2(a0Test2, a1Test2, -1);
    tf::Vector3 point1Test2(0,0, bTest2);

    tf::Vector3 normal2Test2(0,0,-1);
    tf::Vector3 point2Test2(0,0,50);

    planesEqual(normal1Test2, point1Test2,
                normal2Test2, point2Test2);
    ASSERT_TRUE(std::isnan(SurfaceGradientVentPlanner::planeGradientHeading(a0Test2, a1Test2)));

    //TEST 3
    std::vector<tf::Vector3> pointsTest3;
    pointsTest3.emplace_back(0, 0, 60);
    pointsTest3.emplace_back(5, 0, 55);
    pointsTest3.emplace_back(10, 10, 50);
    pointsTest3.emplace_back(10, -10, 50);

    double a0Test3 = 0;
    double a1Test3 = 0;
    double bTest3 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest3, a0Test3, a1Test3, bTest3));

    tf::Vector3 normal1Test3(a0Test3, a1Test3, -1);
    tf::Vector3 point1Test3(0,0, bTest3);

    tf::Vector3 normal2Test3(1,0,1);
    tf::Vector3 point2Test3(0,0,60);

    planesEqual(normal1Test3, point1Test3,
                normal2Test3, point2Test3);
    ASSERT_NEAR(M_PI / 2, SurfaceGradientVentPlanner::planeGradientHeading(a0Test3, a1Test3), 0.00001);

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

    double a0Test4 = 0;
    double a1Test4 = 0;
    double bTest4 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest4, a0Test4, a1Test4, bTest4));


    tf::Vector3 normal1Test4(a0Test4, a1Test4, -1);
    tf::Vector3 point1Test4(0,0, bTest4);

    tf::Vector3 normal2Test4(0,0,-1);
    tf::Vector3 point2Test4(0,0,16.6666666667);

    planesEqual(normal1Test4, point1Test4,
                normal2Test4, point2Test4);
    ASSERT_TRUE(std::isnan(SurfaceGradientVentPlanner::planeGradientHeading(a0Test4, a1Test4)));

    //TEST 5
    std::vector<tf::Vector3> pointsTest5;
    pointsTest5.emplace_back(0, 0, 0);
    pointsTest5.emplace_back(10, 0, 0);
    pointsTest5.emplace_back(-10, 0, 0);

    double a0Test5 = 0;
    double a1Test5 = 0;
    double bTest5 = 0;

    ASSERT_FALSE(SurfaceGradientVentPlanner::fitPlane(pointsTest5, a0Test5, a1Test5, bTest5));

    //TEST 6
    std::vector<tf::Vector3> pointsTest6;
    pointsTest6.emplace_back(0, 0, 40);
    pointsTest6.emplace_back(5, 0, 45);
    pointsTest6.emplace_back(10, 10, 50);
    pointsTest6.emplace_back(10, -10, 50);

    double a0Test6 = 0;
    double a1Test6 = 0;
    double bTest6 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest6, a0Test6, a1Test6, bTest6));

    tf::Vector3 normal1Test6(a0Test6, a1Test6, -1);
    tf::Vector3 point1Test6(0,0, bTest6);

    tf::Vector3 normal2Test6(-1,0,1);
    tf::Vector3 point2Test6(0,0,40);

    planesEqual(normal1Test6, point1Test6,
                normal2Test6, point2Test6);
    ASSERT_NEAR(-M_PI / 2, SurfaceGradientVentPlanner::planeGradientHeading(a0Test6, a1Test6), 0.00001);

    //TEST 7
    std::vector<tf::Vector3> pointsTest7;
    pointsTest7.emplace_back(0, 0, 40);
    pointsTest7.emplace_back(10, 10, 50);
    pointsTest7.emplace_back(-10, 10, 50);

    double a0Test7 = 0;
    double a1Test7 = 0;
    double bTest7 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest7, a0Test7, a1Test7, bTest7));

    tf::Vector3 normal1Test7(a0Test7, a1Test7, -1);
    tf::Vector3 point1Test7(0,0, bTest7);

    tf::Vector3 normal2Test7(0,-1,1);
    tf::Vector3 point2Test7(0,0,40);

    planesEqual(normal1Test7, point1Test7,
                normal2Test7, point2Test7);
    ASSERT_NEAR(-M_PI, SurfaceGradientVentPlanner::planeGradientHeading(a0Test7, a1Test7), 0.00001);

    //TEST 8
    std::vector<tf::Vector3> pointsTest8;
    pointsTest8.emplace_back(0, 0, 60);
    pointsTest8.emplace_back(10, 10, 50);
    pointsTest8.emplace_back(-10, 10, 50);

    double a0Test8 = 0;
    double a1Test8 = 0;
    double bTest8 = 0;

    ASSERT_TRUE(SurfaceGradientVentPlanner::fitPlane(pointsTest8, a0Test8, a1Test8, bTest8));

    tf::Vector3 normal1Test8(a0Test8, a1Test8, -1);
    tf::Vector3 point1Test8(0,0, bTest8);

    tf::Vector3 normal2Test8(0,1,1);
    tf::Vector3 point2Test8(0,0,60);

    planesEqual(normal1Test8, point1Test8,
                normal2Test8, point2Test8);
    ASSERT_NEAR(0, SurfaceGradientVentPlanner::planeGradientHeading(a0Test8, a1Test8), 0.00001);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
