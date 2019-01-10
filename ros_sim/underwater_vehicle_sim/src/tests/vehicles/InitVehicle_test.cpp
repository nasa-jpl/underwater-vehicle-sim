#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "vehicles/Vehicle.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/static_transform_broadcaster.h>

ros::ServiceClient client;

TEST(InitVehicle, InitVehicleTest){
    //Initalize ROS node handle
    ros::NodeHandle n;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    //Initalize vehicle
    Vehicle vehicle1("v1", n, false);
    ros::spinOnce();
    
    geometry_msgs::TransformStamped transformMsgV1;
    tf2::Stamped<tf2::Transform> transformV1;

    bool transformsRecieved = false;

    while(!transformsRecieved)
    {
        try
        {
            transformMsgV1 = buffer.lookupTransform("world", "v1",  
                                                    ros::Time(0));
            tf2::fromMsg(transformMsgV1, transformV1);
            transformsRecieved = true;
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    ASSERT_FLOAT_EQ(521.25, transformV1.getOrigin().getX());
    ASSERT_FLOAT_EQ(-434.5, transformV1.getOrigin().getY());
    ASSERT_FLOAT_EQ(10.5, transformV1.getOrigin().getZ());
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
    ros::init(argc, argv, "init_vehicle_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
