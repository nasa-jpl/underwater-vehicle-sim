#include <gtest/gtest.h>

#include "ros/ros.h"

#include <tf2_ros/static_transform_broadcaster.h>

#include "underwater_vehicle_msgs/Depth.h"

#include "vehicles/DepthModule.h"

std::vector<underwater_vehicle_msgs::Depth> depthMessages;

void imuNoErrorCallback(const underwater_vehicle_msgs::DepthPtr& vel)
{
    depthMessages.push_back(*vel);
}

TEST(DepthModule, ErrorTest)
{
    DepthModule module("depth");
    ModelData modelData;
    ros::Time lastTime(1.234);
    VehicleState state;

    ros::NodeHandle nh;
    ros::Subscriber dataSub = nh.subscribe("depth/data", 1, &imuNoErrorCallback);

    double depth = 150;
    double depthVariance = 1;
    double depthBias = 2;

    std::default_random_engine generator(111);
	std::normal_distribution<double> depthDistribution(0, sqrt(depthVariance));
    double depthError = depthDistribution(generator);

    state.setPositionNED(tf2::Vector3(10, -10, depth));
    module.update(lastTime, state, modelData);
    ros::spinOnce();
 
    ASSERT_EQ(1, depthMessages.size());

    EXPECT_DOUBLE_EQ(1.234, depthMessages[0].header.stamp.toSec());
    EXPECT_DOUBLE_EQ(depth + depthBias + depthError, depthMessages[0].depth);
    EXPECT_DOUBLE_EQ(1, depthMessages[0].variance);
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
    ros::init(argc, argv, "depth_module_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
