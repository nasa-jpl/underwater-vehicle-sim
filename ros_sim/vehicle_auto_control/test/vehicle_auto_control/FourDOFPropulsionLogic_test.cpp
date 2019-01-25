#include <gtest/gtest.h>

#include "ros/ros.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"
/*
TEST(FourDOFPropulsionLogic, TargetBelowSeafloor)
{
    underwater_vehicle_msgs::VehicleData data1;
    data1.sonarDepth = 17.5;
    data1.h = 132.5;

    underwater_vehicle_msgs::VehicleData data2;
    data2.sonarDepth = 10;
    data2.h = 140;

    tf2::Vector3 targetPointNED(10, -20, 300);
    std::string name = "v1";
    ros::Time time(0);

    tf2::Transform transformNearBottom;
    tf2::Vector3 locationNearBottom(15, -20, 132.5); //7.5 m above the seafloor target
    tf2::Quaternion rotationNearBottom;
    rotationNearBottom.setRPY(0, 0, M_PI / 2);
    transformNearBottom.setOrigin(locationNearBottom);
    transformNearBottom.setRotation(rotationNearBottom);
    transformNearBottom = transformNearBottom.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformNearBottom(transformNearBottom, time, name);

    tf2::Transform transformAtBottom;
    tf2::Vector3 locationAtBottom(15, -20, 140); //At exact seafloor target
    tf2::Quaternion rotationAtBottom;
    rotationAtBottom.setRPY(0, 0, M_PI / 2);
    transformAtBottom.setOrigin(locationAtBottom);
    transformAtBottom.setRotation(rotationAtBottom);
    transformAtBottom = transformAtBottom.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformAtBottom(transformAtBottom, time, name);

    FourDOFPropulsionLogic logic;
    geometry_msgs::Twist targetVelocity;
    targetVelocity.linear.x = 0.75;
    targetVelocity.linear.y = 0;
    targetVelocity.linear.z = 0.5;
    targetVelocity.angular.x = 0;
    targetVelocity.angular.y = 0;
    targetVelocity.angular.z = 0.25;
    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());
    logic.setTargetVelocity(targetVelocity);
    
    logic.processNewData(data1);
    logic.goToZ(stampedTransformNearBottom);

    EXPECT_NEAR(targetVelocity.linear.z / 2, twistNearBottom.linear.z, 0.000000001);
    EXPECT_FALSE(logic.isAtZ(stampedTransformNearBottom));

    logic.processNewData(data2);
    EXPECT_TRUE(logic.isAtZ(stampedTransformAtBottom));   
}

TEST(FourDOFPropulsionLogic, GetVerticalTwist)
{
    tf2::Vector3 targetPointNED(10, -20, 30);
    std::string name = "v1";
    ros::Time time(0);

    tf2::Transform transformOutRangePos;
    tf2::Vector3 locationOutRangePos(15, -20, 14); //Above target
    tf2::Quaternion rotationOutRangePos;
    rotationOutRangePos.setRPY(0, 0, M_PI / 2); //Facing East
    transformOutRangePos.setOrigin(locationOutRangePos);
    transformOutRangePos.setRotation(rotationOutRangePos);
    transformOutRangePos = transformOutRangePos.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRangePos(transformOutRangePos, time, name);

    tf2::Transform transformInRangePos;
    tf2::Vector3 locationInRangePos(15, -25, 22.5);  //Above target
    tf2::Quaternion rotationInRangePos;
    rotationInRangePos.setRPY(0, 0, M_PI / 2); //Facing East
    transformInRangePos.setOrigin(locationInRangePos);
    transformInRangePos.setRotation(rotationInRangePos);
    transformInRangePos = transformInRangePos.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformInRangePos(transformInRangePos, time, name);

    tf2::Transform transformOutRangeNeg;
    tf2::Vector3 locationOutRangeNeg(15, -20, 46); //Below target
    tf2::Quaternion rotationOutRangeNeg;
    rotationOutRangeNeg.setRPY(0, 0, -M_PI / 2); //Facing West
    transformOutRangeNeg.setOrigin(locationOutRangeNeg);
    transformOutRangeNeg.setRotation(rotationOutRangeNeg);
    transformOutRangeNeg = transformOutRangeNeg.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRangeNeg(transformOutRangeNeg, time, name);

    tf2::Transform transformInRangeNeg;
    tf2::Vector3 locationInRangeNeg(15, -15, 37.5); //Below target
    tf2::Quaternion rotationInRangeNeg;
    rotationInRangeNeg.setRPY(0, 0, -M_PI / 2); //Facing West
    transformInRangeNeg.setOrigin(locationInRangeNeg);
    transformInRangeNeg.setRotation(rotationInRangeNeg);
    transformInRangeNeg = transformInRangeNeg.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformInRangeNeg(transformInRangeNeg, time, name);

    FourDOFPropulsionLogic logic;
    geometry_msgs::Twist targetVelocity;
    targetVelocity.linear.x = 0.75;
    targetVelocity.linear.y = 0;
    targetVelocity.linear.z = 0.5;
    targetVelocity.angular.x = 0;
    targetVelocity.angular.y = 0;
    targetVelocity.angular.z = 0.25;

    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());
    logic.setTargetVelocity(targetVelocity);

    logic.goToZ(stampedLocationOutRangePos);
    logic.goToZ(stampedTransformInRangePos);
    logic.goToZ(stampedLocationOutRangeNeg);
    logic.goToZ(stampedTransformInRangeNeg);

    EXPECT_NEAR(targetVelocity.linear.z, twistOutRangePos.linear.z, 0.000000001);
    EXPECT_NEAR(targetVelocity.linear.z / 2, twistInRangePos.linear.z, 0.000000001);
    EXPECT_NEAR(-targetVelocity.linear.z, twistOutRangeNeg.linear.z, 0.000000001);
    EXPECT_NEAR(-targetVelocity.linear.z / 2, twistInRangeNeg.linear.z, 0.000000001);
}

TEST(FourDOFPropulsionLogic, GetHorizontalTwist)
{
    tf2::Vector3 targetPointNED(10, -20, 30);
    std::string name = "v1";
    ros::Time time(0);

    tf2::Transform transformOutRange;
    tf2::Vector3 locationOutRange(10, -121, 30); //Due West of target
    tf2::Quaternion rotationOutRange;
    rotationOutRange.setRPY(0, 0, M_PI / 2); //Facing East
    transformOutRange.setOrigin(locationOutRange);
    transformOutRange.setRotation(rotationOutRange);
    transformOutRange = transformOutRange.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRange(transformOutRange, time, name);

    tf2::Transform transformInRange;
    tf2::Vector3 locationInRange(10, -70, 30);  //Due West of target
    tf2::Quaternion rotationInRange;
    rotationInRange.setRPY(0, 0, -M_PI / 2); //Facing West
    transformInRange.setOrigin(locationInRange);
    transformInRange.setRotation(rotationInRange);
    transformInRange = transformInRange.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformInRange(transformInRange, time, name);

    FourDOFPropulsionLogic logic;
    geometry_msgs::Twist targetVelocity;
    targetVelocity.linear.x = 0.75;
    targetVelocity.linear.y = 0;
    targetVelocity.linear.z = 0.5;
    targetVelocity.angular.x = 0;
    targetVelocity.angular.y = 0;
    targetVelocity.angular.z = 0.25;

    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());
    logic.setTargetVelocity(targetVelocity);

    logic.goToXY(stampedLocationOutRange);
    logic.goToXY(stampedTransformInRange);

    EXPECT_NEAR(targetVelocity.linear.x, twistOutRange.linear.x, 0.000000001);
    EXPECT_NEAR(targetVelocity.linear.x / 2, twistInRange.linear.x, 0.000000001);
}

TEST(FourDOFPropulsionLogic, GetRotationalTwist)
{
    tf2::Vector3 targetPointNED(10, -20, 30);
    std::string name = "v1";
    ros::Time time(0);

    tf2::Transform transformOutRangePos;
    tf2::Vector3 locationOutRangePos(15, -20, 30); //Due North of target
    tf2::Quaternion rotationOutRangePos;
    rotationOutRangePos.setRPY(0, 0, M_PI / 2); //Facing East
    transformOutRangePos.setOrigin(locationOutRangePos);
    transformOutRangePos.setRotation(rotationOutRangePos);
    transformOutRangePos = transformOutRangePos.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRangePos(transformOutRangePos, time, name);

    tf2::Transform transformInRangePos;
    tf2::Vector3 locationInRangePos(15, -25, 30);  //NW of target
    tf2::Quaternion rotationInRangePos;
    rotationInRangePos.setRPY(0, 0, M_PI / 2); //Facing East
    transformInRangePos.setOrigin(locationInRangePos);
    transformInRangePos.setRotation(rotationInRangePos);
    transformInRangePos = transformInRangePos.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformInRangePos(transformInRangePos, time, name);

    tf2::Transform transformOutRangeNeg;
    tf2::Vector3 locationOutRangeNeg(15, -20, 30); //Due North of target
    tf2::Quaternion rotationOutRangeNeg;
    rotationOutRangeNeg.setRPY(0, 0, -M_PI / 2); //Facing West
    transformOutRangeNeg.setOrigin(locationOutRangeNeg);
    transformOutRangeNeg.setRotation(rotationOutRangeNeg);
    transformOutRangeNeg = transformOutRangeNeg.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRangeNeg(transformOutRangeNeg, time, name);

    tf2::Transform transformInRangeNeg;
    tf2::Vector3 locationInRangeNeg(15, -15, 30); //NE of target
    tf2::Quaternion rotationInRangeNeg;
    rotationInRangeNeg.setRPY(0, 0, -M_PI / 2); //Facing West
    transformInRangeNeg.setOrigin(locationInRangeNeg);
    transformInRangeNeg.setRotation(rotationInRangeNeg);
    transformInRangeNeg = transformInRangeNeg.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransformInRangeNeg(transformInRangeNeg, time, name);

    FourDOFPropulsionLogic logic;
    geometry_msgs::Twist targetVelocity;
    targetVelocity.linear.x = 0.75;
    targetVelocity.linear.y = 0;
    targetVelocity.linear.z = 0.5;
    targetVelocity.angular.x = 0;
    targetVelocity.angular.y = 0;
    targetVelocity.angular.z = 0.25;

    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());
    logic.setTargetVelocity(targetVelocity);

    logic.goToXY(stampedLocationOutRangePos);
    logic.goToXY(stampedTransformInRangePos);
    logic.goToXY(stampedLocationOutRangeNeg);
    logic.goToXY(stampedTransformInRangeNeg);

    EXPECT_NEAR(targetVelocity.angular.z / 2, twistOutRangePos.angular.z, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.z / 4, twistInRangePos.angular.z, 0.000000001);
    EXPECT_NEAR(-targetVelocity.angular.z / 2, twistOutRangeNeg.angular.z, 0.000000001);
    EXPECT_NEAR(-targetVelocity.angular.z / 4, twistInRangeNeg.angular.z, 0.000000001);
}

TEST(FourDOFPropulsionLogic, AtLocationTest)
{
    tf2::Vector3 targetPointNED(10, -20, 30);
    std::string name = "v1";
    ros::Time time(0);
    tf2::Transform transform1;
    tf2::Vector3 location1(5.1, -20, 30.9);
    tf2::Quaternion rotation1;
    rotation1.setRPY(0, 0, 0);
    transform1.setOrigin(location1);
    transform1.setRotation(rotation1);
    transform1 = transform1.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransform1(transform1, time, name);

    tf2::Transform transform2;
    tf2::Vector3 location2(4.9, -20, 31.1);
    tf2::Quaternion rotation2;
    rotation2.setRPY(0, 0, 0);
    transform2.setOrigin(location2);
    transform2.setRotation(rotation2);
    transform2 = transform2.inverse();  //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedTransform2(transform2, time, name);

    FourDOFPropulsionLogic logic;
    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());

    EXPECT_TRUE(logic.isAtXY(stampedTransform1));
    EXPECT_FALSE(logic.isAtXY(stampedTransform2));

    EXPECT_TRUE(logic.isAtZ(stampedTransform1));
    EXPECT_FALSE(logic.isAtZ(stampedTransform2));
}
*/
TEST(FourDOFPropulsionLogic, StopTwist)
{
  /*  tf2::Vector3 targetPointNED(10, -20, 30);
    std::string name = "v1";
    ros::Time time(0);

    tf2::Transform transformOutRangePos;
    tf2::Vector3 locationOutRangePos(10, 100, 10); //Due North of target
    tf2::Quaternion rotationOutRangePos;
    rotationOutRangePos.setRPY(0, 0, M_PI); //Facing East
    transformOutRangePos.setOrigin(locationOutRangePos);
    transformOutRangePos.setRotation(rotationOutRangePos);
    transformOutRangePos = transformOutRangePos.inverse(); //The inverse is used by FourDOFPropulsionLogic
    tf2::Stamped<tf2::Transform> stampedLocationOutRangePos(transformOutRangePos, time, name);

    FourDOFPropulsionLogic logic;
    geometry_msgs::Twist targetVelocity;
    targetVelocity.linear.x = 0.75;
    targetVelocity.linear.y = 0;
    targetVelocity.linear.z = 0.5;
    targetVelocity.angular.x = 0;
    targetVelocity.angular.y = 0;
    targetVelocity.angular.z = 0.25;

    logic.setTargetXY(targetPointNED.getX(), targetPointNED.getY());
    logic.setTargetZ(targetPointNED.getZ());
    logic.setTargetVelocity(targetVelocity);

    //Set XYZ twist
    logic.goToXY(stampedLocationOutRangePos);
    logic.goToZ(stampedLocationOutRangePos);

    logic.stopXY();

    //Set XYZ twist
    logic.goToXY(stampedLocationOutRangePos);
    logic.goToZ(stampedLocationOutRangePos);

    logic.stopZ();

    EXPECT_NEAR(0, stopTwistXY.linear.x, 0.000000001);
    EXPECT_NEAR(0, stopTwistXY.linear.y, 0.000000001);
    EXPECT_NEAR(targetVelocity.linear.z, stopTwistXY.linear.z, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.x, stopTwistXY.angular.x, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.y, stopTwistXY.angular.y, 0.000000001);
    EXPECT_NEAR(0, stopTwistXY.angular.z, 0.000000001);

    EXPECT_NEAR(targetVelocity.linear.x, stopTwistZ.linear.x, 0.000000001);
    EXPECT_NEAR(targetVelocity.linear.y, stopTwistZ.linear.y, 0.000000001);
    EXPECT_NEAR(0, stopTwistZ.linear.z, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.x, stopTwistZ.angular.x, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.y, stopTwistZ.angular.y, 0.000000001);
    EXPECT_NEAR(targetVelocity.angular.z / 2, stopTwistZ.angular.z, 0.000000001);*/
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
