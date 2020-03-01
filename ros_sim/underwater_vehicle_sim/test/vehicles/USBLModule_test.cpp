#include <gtest/gtest.h>

#include "ros/ros.h"

#include <tf2_ros/static_transform_broadcaster.h>

#include "underwater_vehicle_msgs/USBL.h"
#include "vehicles/USBLModule.h"

using namespace ocean_models;

std::vector<underwater_vehicle_msgs::USBL> validRangeMessages;
std::vector<underwater_vehicle_msgs::USBL> biasAndRandomErrorMessages;
std::vector<underwater_vehicle_msgs::USBL> badRangeMessages;

void validRangeCallback(const underwater_vehicle_msgs::USBLPtr& vel)
{
    validRangeMessages.push_back(*vel);
}

void biasAndRandomErrorCallback(const underwater_vehicle_msgs::USBLPtr& vel)
{
    biasAndRandomErrorMessages.push_back(*vel);
}

void badRangeCallback(const underwater_vehicle_msgs::USBLPtr& vel)
{
    badRangeMessages.push_back(*vel);
}
/*
TEST(USBLModule, TestValidRange)
{
    USBLModule module("valid_range_usbl");
    ModelData modelData;
    ros::Time lastTime(0);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("valid_range_usbl/data", 1, &validRangeCallback);

    //Valid Range and Bearing
    state.setPositionNED(tf2::Vector3(0, 0, 0));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    //Valid Range, Invalid Bearing - XY Direction
    //100 m from beacon at 45 degree bearing
    state.setPositionNED(tf2::Vector3(81.4177848998, 66.4177848998, 20));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    //Valid Range, Invalid Bearing - Z Direction
    //200 m from beacon down
    state.setPositionNED(tf2::Vector3(11, -5, 120));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    //Invalid Range and Bearing - XY Direction
    //200 m from beacon at 45 degree bearing
    state.setPositionNED(tf2::Vector3(152.128463018, 137.128463018, 20));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    //Invalid Range and Bearing - Z Direction
    //200 m from beacon down
    state.setPositionNED(tf2::Vector3(11, -5, 220));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    ASSERT_EQ(5, validRangeMessages.size());
    EXPECT_FALSE(std::isnan(validRangeMessages[0].range));
    EXPECT_FALSE(std::isnan(validRangeMessages[0].bearing));

    EXPECT_FALSE(std::isnan(validRangeMessages[1].range));
    EXPECT_TRUE(std::isnan(validRangeMessages[1].bearing));

    EXPECT_FALSE(std::isnan(validRangeMessages[2].range));
    EXPECT_TRUE(std::isnan(validRangeMessages[2].bearing));

    EXPECT_TRUE(std::isnan(validRangeMessages[3].range));
    EXPECT_TRUE(std::isnan(validRangeMessages[3].bearing));

    EXPECT_TRUE(std::isnan(validRangeMessages[4].range));
    EXPECT_TRUE(std::isnan(validRangeMessages[4].bearing));
}

TEST(USBLModule, TestBiasAndRandomError)
{
    USBLModule module("error_usbl");
    ModelData modelData;
    ros::Time lastTime(0);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("error_usbl/data", 1, &biasAndRandomErrorCallback);

    std::default_random_engine generator(111);

    std::uniform_real_distribution<double> badRangeRandom(0, 1.0);
    std::normal_distribution<double> rangeDistribution(0, 7.07106781187 * 0.005);
    std::normal_distribution<double> bearingDistribution(0, 0.0174533);

    //Parameters
    double trueRange = 7.07106781187; //5 * sqrt(2)
    double bearing45 = 0.78539816339; //45 degrees in rad
    double bearingBias = 0.0872665;
    double rangeBias = 5;

    //Re-create random number generation that should be used in the module
    double badRangeProp = badRangeRandom(generator);
    double rangeError = rangeDistribution(generator);
    double bearingError = bearingDistribution(generator);

    state.setPositionNED(tf2::Vector3(15, 0, 20));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    ASSERT_EQ(1, biasAndRandomErrorMessages.size());
    EXPECT_FLOAT_EQ(trueRange + rangeBias + rangeError, biasAndRandomErrorMessages[0].range);
    EXPECT_FLOAT_EQ(bearing45 + bearingBias + bearingError, biasAndRandomErrorMessages[0].bearing);
}
*/
TEST(USBLModule, TestBadRange){
    USBLModule module("bad_range_usbl");
    ModelData modelData;
    ros::Time lastTime(0);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("bad_range_usbl/data", 1, &badRangeCallback);

    std::default_random_engine generator(111);

    //Parameters
    double trueRange = 7.07106781187; //5 * sqrt(2)
    double bearing45 = 0.78539816339; //45 degrees in rad
    double bearingBias = 0.0872665;
    double rangeBias = 5;

    std::uniform_real_distribution<double> badRangeRandom(0, 1.0);
    std::uniform_real_distribution<double> badRangeDistribution(-100, 100);
    std::normal_distribution<double> rangeDistribution(0, trueRange * 0.005);
    std::normal_distribution<double> bearingDistribution(0, 0.0174533);

    //Re-create random number generation that should be used in the module
    badRangeRandom(generator);  //Here so random number gen follows the module
    double badRangeError1 = badRangeDistribution(generator);
    double bearingError1 = bearingDistribution(generator);

    badRangeRandom(generator); //Here so random number gen follows the module
    double rangeError2 = rangeDistribution(generator);
    double bearingError2 = bearingDistribution(generator);

    state.setPositionNED(tf2::Vector3(15, 0, 20));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    state.setPositionNED(tf2::Vector3(15, -10, 20));
    module.update(lastTime, state, modelData);
    ros::spinOnce();

    ASSERT_EQ(2u, badRangeMessages.size());
    EXPECT_FLOAT_EQ(trueRange + badRangeError1, badRangeMessages[0].range); //Bias not applied when bad range
    EXPECT_FLOAT_EQ(bearing45 + bearingBias + bearingError1, badRangeMessages[0].bearing); //Bearing bias still a

    EXPECT_FLOAT_EQ(trueRange + rangeBias + rangeError2, badRangeMessages[1].range);
    EXPECT_FLOAT_EQ(-bearing45 + bearingBias + bearingError2, badRangeMessages[1].bearing);

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
    ros::init(argc, argv, "usbl_module_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
