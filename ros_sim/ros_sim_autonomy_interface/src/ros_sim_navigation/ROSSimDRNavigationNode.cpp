#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

#include "tf2_ros/transform_listener.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "underwater_autonomy/navigation/DeadReckoningNavigationFilter.h"

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

using namespace underwater_autonomy;

ros::Publisher posePublisher;

std::unique_ptr<DeadReckoningNavigationFilter> filter;
ros::Timer updateTimer;

void publishPose() {
    VehiclePose pose = filter->getPoseEstimation();

    Eigen::Vector3d position = pose.getPosition();
    Eigen::Quaterniond orientation = pose.getOrientation();
    Eigen::Vector3d linearVelocity = pose.getLinearVelocity();
    Eigen::Vector3d angularVelocity = pose.getAngularVelocity();
    Eigen::Matrix<double,6,6> poseCovariance = pose.getPoseCovariance();
    Eigen::Matrix<double,6,6> twistCovariance = pose.getTwistCovariance();

    nav_msgs::Odometry odoMsg;
    odoMsg.header.stamp = ros::Time::now();
    odoMsg.header.frame_id = "world_ned"; //pose frame
    odoMsg.child_frame_id = "world_ned"; //twist frame

    odoMsg.pose.pose.position.x = position[0];
    odoMsg.pose.pose.position.y = position[1];
    odoMsg.pose.pose.position.z = position[2];

    odoMsg.pose.pose.orientation.x = orientation.x();
    odoMsg.pose.pose.orientation.y = orientation.y();
    odoMsg.pose.pose.orientation.z = orientation.z();
    odoMsg.pose.pose.orientation.w = orientation.w();

    for(uint i = 0; i < 6; i++) {
        for(uint j = 0; j < 6; j++) {
            odoMsg.pose.covariance[i*6 + j] = poseCovariance(i,j);
        }
    }

    odoMsg.twist.twist.linear.x = linearVelocity[0];
    odoMsg.twist.twist.linear.y = linearVelocity[1];
    odoMsg.twist.twist.linear.z = linearVelocity[2];

    odoMsg.twist.twist.angular.x = angularVelocity[0];
    odoMsg.twist.twist.angular.y = angularVelocity[1];
    odoMsg.twist.twist.angular.z = angularVelocity[2];

    for(uint i = 0; i < 6; i++) {
        for(uint j = 0; j < 6; j++) {
            odoMsg.twist.covariance[i*6 + j] = twistCovariance(i,j);
        }
    }
    posePublisher.publish(odoMsg);
}

void timerCallback(const ros::TimerEvent&) {
    filter->update();
    publishPose();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_dr_navigation");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    vehicleInfoClient.waitForExistence();
    underwater_vehicle_msgs::GetVehicleInfo getInfo;
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);
    std::shared_ptr<ROSSimVehicleInterface> interface = std::make_shared<ROSSimVehicleInterface>(info);

    Eigen::Vector3d startPosition(info.getStartX(), info.getStartY(), info.getStartZ());
    Eigen::Quaterniond startOrientation;
    startOrientation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    VehiclePose startPose(startPosition, startOrientation);
    Eigen::Matrix<double,6,6> poseCovariance = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> twistCovariance = Eigen::Matrix<double,6,6>::Zero();
    twistCovariance(0,0) = 2;
    twistCovariance(1,1) = 2;
    twistCovariance(5,5) = 0;

    startPose.setPoseCovariance(poseCovariance);
    startPose.setTwistCovariance(twistCovariance);

    std::string configFilename;
    nhPriv.getParam("config_file", configFilename);
    ConfigurationFile config(configFilename);
    DeadReckoningNavigationFilter::Parameters parameters(config);

    filter = std::unique_ptr<DeadReckoningNavigationFilter>(new DeadReckoningNavigationFilter(interface, parameters));
    filter->setPose(startPose);

    float hertz;
    if(!nhPriv.getParam("hertz", hertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    posePublisher = nh.advertise<nav_msgs::Odometry>("dead_reckoning_nav/pose", 10);
    updateTimer = nh.createTimer(ros::Duration(1/hertz), timerCallback);
    ros::spin();
}