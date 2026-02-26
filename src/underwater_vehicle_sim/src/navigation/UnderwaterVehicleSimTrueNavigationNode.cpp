#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

#include "tf2_ros/transform_listener.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

ros::Publisher posePublisher;

tf2_ros::Buffer buffer;

ros::Timer tfTimer;
ros::Time lastTFTime;

bool lastPoseValid;
Eigen::Vector3d lastPosition;
Eigen::Quaterniond lastOrientation;    
Eigen::Vector3d lastLinearVelocity;
Eigen::Vector3d lastAngularVelocity;

std::string vehicleName;

void publishPose() {
    nav_msgs::Odometry odoMsg;
    odoMsg.header.stamp = ros::Time::now();
    odoMsg.header.frame_id = "world_ned"; //pose frame
    odoMsg.child_frame_id = "world_ned"; //twist frame

    odoMsg.pose.pose.position.x = lastPosition[0];
    odoMsg.pose.pose.position.y = lastPosition[1];
    odoMsg.pose.pose.position.z = lastPosition[2];

    odoMsg.pose.pose.orientation.x = lastOrientation.x();
    odoMsg.pose.pose.orientation.y = lastOrientation.y();
    odoMsg.pose.pose.orientation.z = lastOrientation.z();
    odoMsg.pose.pose.orientation.w = lastOrientation.w();

    for(unsigned int i = 0; i < 36; i++)
    {
        odoMsg.pose.covariance[i] = 0;
    }

    odoMsg.twist.twist.linear.x = lastLinearVelocity[0];
    odoMsg.twist.twist.linear.y = lastLinearVelocity[1];
    odoMsg.twist.twist.linear.z = lastLinearVelocity[2];

    odoMsg.twist.twist.angular.x = lastAngularVelocity[0];
    odoMsg.twist.twist.angular.y = lastAngularVelocity[1];
    odoMsg.twist.twist.angular.z = lastAngularVelocity[2];

    for(unsigned int i = 0; i < 36; i++)
    {
        odoMsg.twist.covariance[i] = 0;
    }

    posePublisher.publish(odoMsg);
}

void getPoseFromTF() {

    geometry_msgs::TransformStamped transformMsg;
	try
    {
        if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(1.0)))
        {
            transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));

            Eigen::Vector3d position (transformMsg.transform.translation.x,
                                      transformMsg.transform.translation.y,
                                      transformMsg.transform.translation.z);
            Eigen::Quaterniond orientation(transformMsg.transform.rotation.w,
                                           transformMsg.transform.rotation.x,
                                           transformMsg.transform.rotation.y,
                                           transformMsg.transform.rotation.z);


            if(!lastPoseValid) {
                lastPosition = position;
                lastOrientation = orientation;
                lastTFTime = transformMsg.header.stamp;
                lastPoseValid = true;
            }

            double timeDelta = (transformMsg.header.stamp - lastTFTime).toSec();
            if(timeDelta > 0) {
                //Extract new lateral velocity
                Eigen::Vector3d linearVelocity = (position - lastPosition) / timeDelta;
                linearVelocity = orientation.inverse() * linearVelocity; //Rotate linear velocity into body frame from world frame

                //Extract new rotational velocity
                Eigen::Quaterniond rotation = orientation  * lastOrientation.inverse();

                Eigen::AngleAxisd rotationAA(rotation);
                Eigen::Vector3d angularVelocity = rotationAA.axis() * (rotationAA.angle() / timeDelta);
                angularVelocity = orientation.inverse() * angularVelocity; //Rotate angular velocity into body frame from world frame

                lastPosition = position;
                lastOrientation = orientation;
                lastLinearVelocity = linearVelocity;
                lastAngularVelocity = angularVelocity;
                lastTFTime = transformMsg.header.stamp;
            } 

        }
    }
    catch(tf2::TransformException ex)
    {
        throw std::move(ex);
    }  
}

void timerCallback(const ros::TimerEvent&) {
    getPoseFromTF();
    publishPose();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_true_navigation");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    tf2_ros::TransformListener listener(buffer);

    vehicleName = nh.getNamespace().substr(1);

    float hertz;
    if(!nhPriv.getParam("hertz", hertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    posePublisher = nh.advertise<nav_msgs::Odometry>("true_nav/pose", 10);
    lastPoseValid = false;

    tfTimer = nh.createTimer(ros::Duration(1/hertz), timerCallback);
    ros::spin();
}