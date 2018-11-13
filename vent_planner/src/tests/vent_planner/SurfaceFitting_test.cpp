#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <map>

#include <eigen3/Eigen/Dense>
#include "vent_planner/util/Plane.h"

TEST(SurfaceFitting, PlaneFit)
{
    std::map<std::string, double> zeroMap = {{"plume",0}};
    std::map<std::string, double> fortyMap = {{"plume",40}};
    std::map<std::string, double> fortyFiveMap = {{"plume",45}};
    std::map<std::string, double> fiftyMap = {{"plume",50}};
    std::map<std::string, double> fiftyFiveMap = {{"plume",55}};
    std::map<std::string, double> sixtyMap = {{"plume",60}};
    //TEST 1
    std::vector<PlannerData> pointsTest1;
    pointsTest1.emplace_back(0, VehiclePose(0,0,0), zeroMap);
    pointsTest1.emplace_back(0, VehiclePose(10,10,0), zeroMap);
    pointsTest1.emplace_back(0, VehiclePose(10,-10,0), zeroMap);

    Plane fitPlane1 = Plane::fitPlaneToPoints(pointsTest1);

    Eigen::Vector3d normalResultTest1(0,0,-1);
    Eigen::Vector3d pointResultTest1(0,0,0);
    Plane resultPlane1(normalResultTest1, pointResultTest1);

    ASSERT_TRUE(fitPlane1.equals(resultPlane1));
    ASSERT_TRUE(std::isnan(fitPlane1.getHeightGradientHeading()));

    //TEST 2
    std::vector<PlannerData> pointsTest2;
    pointsTest2.emplace_back(0, VehiclePose(0,0,0), fiftyMap);
    pointsTest2.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest2.emplace_back(0, VehiclePose(10,-10,0), fiftyMap);

    Plane fitPlane2 = Plane::fitPlaneToPoints(pointsTest2);

    Eigen::Vector3d normalResultTest2(0,0,-1);
    Eigen::Vector3d pointResultTest2(0,0,50);
    Plane resultPlane2(normalResultTest2, pointResultTest2);

    ASSERT_TRUE(fitPlane2.equals(resultPlane2));
    ASSERT_TRUE(std::isnan(fitPlane2.getHeightGradientHeading()));

    //TEST 3
    std::vector<PlannerData> pointsTest3;
    pointsTest3.emplace_back(0, VehiclePose(0,0,0), sixtyMap);
    pointsTest3.emplace_back(0, VehiclePose(5,0,0), fiftyFiveMap);
    pointsTest3.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest3.emplace_back(0, VehiclePose(10,-10,0), fiftyMap);

    Plane fitPlane3 = Plane::fitPlaneToPoints(pointsTest3);

    Eigen::Vector3d normalResultTest3(1,0,1);
    Eigen::Vector3d pointResultTest3(0,0,60);
    Plane resultPlane3(normalResultTest3, pointResultTest3);

    ASSERT_TRUE(fitPlane3.equals(resultPlane3));
    ASSERT_NEAR(M_PI / 2, fitPlane3.getHeightGradientHeading(), 0.00001);

    //TEST 4
    std::vector<PlannerData> pointsTest4;
    pointsTest4.emplace_back(0, VehiclePose(0,0,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(10,10,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(10,-10,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(0,0,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(10,10,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(10,-10,0), zeroMap);
    pointsTest4.emplace_back(0, VehiclePose(0,0,0), fiftyMap);
    pointsTest4.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest4.emplace_back(0, VehiclePose(10,-10,0), fiftyMap);

    Plane fitPlane4 = Plane::fitPlaneToPoints(pointsTest4);

    Eigen::Vector3d normalResultTest4(0,0,-1);
    Eigen::Vector3d pointResultTest4(0,0,16.6666666667);
    Plane resultPlane4(normalResultTest4, pointResultTest4);

    ASSERT_TRUE(fitPlane4.equals(resultPlane4));
    ASSERT_TRUE(std::isnan(fitPlane4.getHeightGradientHeading()));

    //TEST 5
    std::vector<PlannerData> pointsTest5;
    pointsTest5.emplace_back(0, VehiclePose(0,0,0), zeroMap);
    pointsTest5.emplace_back(0, VehiclePose(10,0,0), zeroMap);
    pointsTest5.emplace_back(0, VehiclePose(-10,0,0), zeroMap);

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
    std::vector<PlannerData> pointsTest6;
    pointsTest6.emplace_back(0, VehiclePose(0,0,0), fortyMap);
    pointsTest6.emplace_back(0, VehiclePose(5,0,0), fortyFiveMap);
    pointsTest6.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest6.emplace_back(0, VehiclePose(10,-10,0), fiftyMap);

    Plane fitPlane6 = Plane::fitPlaneToPoints(pointsTest6);

    Eigen::Vector3d normalResultTest6(-1,0,1);
    Eigen::Vector3d pointResultTest6(0,0,40);
    Plane resultPlane6(normalResultTest6, pointResultTest6);

    ASSERT_TRUE(fitPlane6.equals(resultPlane6));
    ASSERT_NEAR(-M_PI / 2, fitPlane6.getHeightGradientHeading(), 0.00001);

    //TEST 7
    std::vector<PlannerData> pointsTest7;
    pointsTest7.emplace_back(0, VehiclePose(0,0,0), fortyMap);
    pointsTest7.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest7.emplace_back(0, VehiclePose(-10,10,0), fiftyMap);

    Plane fitPlane7 = Plane::fitPlaneToPoints(pointsTest7);

    Eigen::Vector3d normalResultTest7(0,-1,1);
    Eigen::Vector3d pointResultTest7(0,0,40);
    Plane resultPlane7(normalResultTest7, pointResultTest7);

    ASSERT_TRUE(fitPlane7.equals(resultPlane7));
    ASSERT_NEAR(-M_PI, fitPlane7.getHeightGradientHeading(), 0.00001);

    //TEST 8
    std::vector<PlannerData> pointsTest8;
    pointsTest8.emplace_back(0, VehiclePose(0,0,0), sixtyMap);
    pointsTest8.emplace_back(0, VehiclePose(10,10,0), fiftyMap);
    pointsTest8.emplace_back(0, VehiclePose(-10,10,0), fiftyMap);

    Plane fitPlane8 = Plane::fitPlaneToPoints(pointsTest8);

    Eigen::Vector3d normalResultTest8(0,1,1);
    Eigen::Vector3d pointResultTest8(0,0,60);
    Plane resultPlane8(normalResultTest8, pointResultTest8);

    ASSERT_TRUE(fitPlane8.equals(resultPlane8));
    ASSERT_NEAR(0, fitPlane8.getHeightGradientHeading(), 0.00001);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
