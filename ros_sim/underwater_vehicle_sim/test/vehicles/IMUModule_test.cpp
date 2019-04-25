#include <gtest/gtest.h>

#include "ros/ros.h"

#include <tf2_ros/static_transform_broadcaster.h>

#include "sensor_msgs/Imu.h"

#include "vehicles/IMUModule.h"

using namespace ocean_models;

std::vector<sensor_msgs::Imu> errorMessages;
std::vector<sensor_msgs::Imu> noErrorMessages;

void imuErrorCallback(const sensor_msgs::ImuPtr& vel)
{
    errorMessages.push_back(*vel);
}

void imuNoErrorCallback(const sensor_msgs::ImuPtr& vel)
{
    noErrorMessages.push_back(*vel);
}

TEST(IMUModule, NoErrorTest)
{
    IMUModule module("no_error_imu");
    ModelData modelData;
    ros::Time lastTime(0);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("no_error_imu/data", 1, &imuNoErrorCallback);

    tf2::Quaternion rotation1;
    rotation1.setRPY(0.523599, -0.523599, -0.785398); //30, -30, -45
    state.setRotationNED(rotation1);
    state.setAngularVelocityNED(tf2::Vector3(-0.1, 0.1, 0.3));
    module.update(lastTime, state, modelData);
    ros::spinOnce();
 
    ASSERT_EQ(1, noErrorMessages.size());
    tf2::Quaternion q0(noErrorMessages[0].orientation.x,
                       noErrorMessages[0].orientation.y,
                       noErrorMessages[0].orientation.z,
                       noErrorMessages[0].orientation.w);
    double roll0, pitch0, yaw0;
    tf2::Matrix3x3(q0).getRPY(roll0, pitch0, yaw0);
    EXPECT_DOUBLE_EQ(0.523599, roll0);
    EXPECT_DOUBLE_EQ(-0.523599, pitch0);
    EXPECT_DOUBLE_EQ(-0.785398, yaw0);
    EXPECT_DOUBLE_EQ(-0.1, noErrorMessages[0].angular_velocity.x);
    EXPECT_DOUBLE_EQ(0.1, noErrorMessages[0].angular_velocity.y);
    EXPECT_DOUBLE_EQ(0.3, noErrorMessages[0].angular_velocity.z);
}

TEST(IMUModule, ErrorTest)
{
    IMUModule module("error_imu");
    ModelData modelData;
    ros::Time lastTime(0);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("error_imu/data", 1, &imuErrorCallback);
    std::default_random_engine generator(111);

    double roll = 0.785398;
    double pitch = -0.785398;
    double yaw = 0.1;
    std::vector<double> angularVel = {-0.01, 0.02, 0.03};

    double angularVelRandomVariance = 0.09;
    std::vector<double> angularVelBiasError = {-0.01, 0.02, 0.03};
    double headingRandomVariance = 0.04;
    double yawBiasError = 0.05;
    double rollPitchRandomVariance = 0.06;
    double rollBiasError = 0.07;
    double pitchBiasError = 0.08;

    std::normal_distribution<double> headingDist(0, sqrt(headingRandomVariance));
	std::normal_distribution<double> tiltDist(0, sqrt(rollPitchRandomVariance));
	std::normal_distribution<double> rotationDistribution(0, sqrt(angularVelRandomVariance));

    std::vector<double> angularVelRandomError;
    angularVelRandomError.push_back(rotationDistribution(generator));
    angularVelRandomError.push_back(rotationDistribution(generator));
    angularVelRandomError.push_back(rotationDistribution(generator));

    double rollRandomError = tiltDist(generator);
    double pitchRandomError = tiltDist(generator);
    double yawRandomError = headingDist(generator);

    tf2::Quaternion rotation1;
    rotation1.setRPY(roll, pitch, yaw);

    state.setRotationNED(rotation1);
    state.setAngularVelocityNED(tf2::Vector3(-0.01, 0.02, 0.03));
    module.update(lastTime, state, modelData);
    ros::spinOnce();
 
    ASSERT_EQ(1, errorMessages.size());
    tf2::Quaternion q0(errorMessages[0].orientation.x,
                       errorMessages[0].orientation.y,
                       errorMessages[0].orientation.z,
                       errorMessages[0].orientation.w);
    double roll0, pitch0, yaw0;
    tf2::Matrix3x3(q0).getRPY(roll0, pitch0, yaw0);
    EXPECT_DOUBLE_EQ(roll + rollRandomError + rollBiasError, roll0);
    EXPECT_DOUBLE_EQ(pitch + pitchRandomError + pitchBiasError, pitch0);
    EXPECT_DOUBLE_EQ(yaw + yawRandomError + yawBiasError, yaw0);

    tf2::Quaternion test;
    test.setRPY(roll + rollRandomError + rollBiasError,pitch + pitchRandomError + pitchBiasError,yaw + yawRandomError + yawBiasError);

    EXPECT_DOUBLE_EQ(angularVel[0] + angularVelRandomError[0] + angularVelBiasError[0], 
                     errorMessages[0].angular_velocity.x);
    EXPECT_DOUBLE_EQ(angularVel[1] + angularVelRandomError[1] + angularVelBiasError[1],
                     errorMessages[0].angular_velocity.y);
    EXPECT_DOUBLE_EQ(angularVel[2] + angularVelRandomError[2] + angularVelBiasError[2], 
                     errorMessages[0].angular_velocity.z);
}

//Had issues doing this in the roslaunch file for this test. Not sure why.
//Normally this can be included in the roslaunch file with the following
//<node pkg="tf2_ros" type="static_transform_publisher" name="ned_publisher" args="0 0 0 1.57 0 3.14 world world_ned"/>
void broadcastStaticTransform()
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "world_ned";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, M_PI / 2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "imu_module_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
