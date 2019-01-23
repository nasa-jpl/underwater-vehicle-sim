#include <gtest/gtest.h>

#include "ros/ros.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_navigation/ROSSimNavigationFilter.h"


#include <iostream>

std::vector<VehiclePose> poses;

void filterPoseCallback(const nav_msgs::Odometry::ConstPtr& odo)
{
    Eigen::Vector3d position(odo->pose.pose.position.x,
                             odo->pose.pose.position.y,
                             odo->pose.pose.position.z);

    Eigen::Quaterniond orientation(odo->pose.pose.orientation.w,
                                   odo->pose.pose.orientation.x,
                                   odo->pose.pose.orientation.y,
                                   odo->pose.pose.orientation.z);

    Eigen::Matrix<double,6,6> covariance;
    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            covariance(i, j) = odo->pose.covariance[(i * 6) + j];
        }
    }

    Eigen::Vector3d linearVelocity(odo->twist.twist.linear.x,
                                   odo->twist.twist.linear.y,
                                   odo->twist.twist.linear.z);

    Eigen::Vector3d angularVelocity(odo->twist.twist.angular.x,
                                    odo->twist.twist.angular.y,
                                    odo->twist.twist.angular.z);

    Eigen::Matrix<double,6,6> twistCovariance;
    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            twistCovariance(i, j) = odo->twist.covariance[(i * 6) + j];
        }
    }

    VehiclePose vehiclePose;
    vehiclePose.setPosition(position);
    vehiclePose.setOrientation(orientation);
    vehiclePose.setPoseCovariance(covariance);

    vehiclePose.setLinearVelocity(linearVelocity);
    vehiclePose.setAngularVelocity(angularVelocity);
    vehiclePose.setTwistCovariance(twistCovariance);

    poses.push_back(vehiclePose);
}

TEST(ROSSimNavigationFilter, TrueNavigation)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static tf2_ros::TransformBroadcaster br;

    ros::NodeHandle nhRoot;
    ros::NodeHandle nhNav("navigation");

    std::vector<VehiclePose> targetPoses;
    ros::Subscriber poseSub = nhNav.subscribe("v1/v1_true", 10, &filterPoseCallback);


    underwater_vehicle_msgs::GetVehicleInfo infoSrv;
    infoSrv.request.name = "v1";

    infoSrv.response.propModuleType = "None";
    infoSrv.response.propModuleName = "None";

    infoSrv.response.startX = -20.2;
    infoSrv.response.startY = 30.1;
    infoSrv.response.startZ = 10.5;

    VehicleInfo info(infoSrv);
    std::string filterName = "v1_true";
    ROSSimNavigationFilter filter = ROSSimNavigationFilter::createNavigationFilter(nhRoot,
                                                                                   nhNav,
                                                                                   filterName,
                                                                                   info);


    Eigen::Matrix3d m0;
    m0 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    targetPoses.emplace_back(Eigen::Vector3d(infoSrv.response.startX,
                                      infoSrv.response.startY,
                                      infoSrv.response.startZ),
                                      Eigen::Quaterniond(m0));

    Eigen::Matrix3d m1;
    m1 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ());
    targetPoses.emplace_back(Eigen::Vector3d(1, 2, 3),
                      Eigen::Quaterniond(m1));

    Eigen::Matrix3d m2;
    m2 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    targetPoses.emplace_back(Eigen::Vector3d(1.1, 2.1, 3.1),
                      Eigen::Quaterniond(m2));

    filter.publishPose();

    geometry_msgs::TransformStamped transformStamped1;
	transformStamped1.header.stamp = ros::Time::now();
  	transformStamped1.header.frame_id = "world_ned";
  	transformStamped1.child_frame_id = "v1";

	Eigen::Vector3d position1 = targetPoses[1].getPosition();
	transformStamped1.transform.translation.x = position1[0];
	transformStamped1.transform.translation.y = position1[1];
	transformStamped1.transform.translation.z = position1[2];

	Eigen::Quaterniond rotation1 = targetPoses[1].getOrientation();
	transformStamped1.transform.rotation.x = rotation1.x();
	transformStamped1.transform.rotation.y = rotation1.y();
	transformStamped1.transform.rotation.z = rotation1.z();
	transformStamped1.transform.rotation.w = rotation1.w();
  	br.sendTransform(transformStamped1);
    ros::spinOnce();

    filter.update();
    filter.publishPose();

    ros::Duration(1).sleep();

    geometry_msgs::TransformStamped transformStamped2;
	transformStamped2.header.stamp = ros::Time::now();
  	transformStamped2.header.frame_id = "world_ned";
  	transformStamped2.child_frame_id = "v1";

	Eigen::Vector3d position2 = targetPoses[2].getPosition();
	transformStamped2.transform.translation.x = position2[0];
	transformStamped2.transform.translation.y = position2[1];
	transformStamped2.transform.translation.z = position2[2];

	Eigen::Quaterniond rotation2 = targetPoses[2].getOrientation();
	transformStamped2.transform.rotation.x = rotation2.x();
	transformStamped2.transform.rotation.y = rotation2.y();
	transformStamped2.transform.rotation.z = rotation2.z();
	transformStamped2.transform.rotation.w = rotation2.w();
  	br.sendTransform(transformStamped2);
    ros::spinOnce();

    filter.update();
    filter.publishPose();

    //Wait for the poses to be sent over the ros topic
    while(poses.size() != 3)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ros::spinOnce();
    }
    
    targetPoses[2].setLinearVelocity(Eigen::Vector3d((targetPoses[2].getPosition()[0] - targetPoses[1].getPosition()[0]) / (transformStamped2.header.stamp.toSec() - transformStamped1.header.stamp.toSec()),
                                                     (targetPoses[2].getPosition()[1] - targetPoses[1].getPosition()[1]) / (transformStamped2.header.stamp.toSec() - transformStamped1.header.stamp.toSec()),
                                                     (targetPoses[2].getPosition()[2] - targetPoses[1].getPosition()[2]) / (transformStamped2.header.stamp.toSec() - transformStamped1.header.stamp.toSec())));
    targetPoses[2].setAngularVelocity(Eigen::Vector3d(0, 0, (M_PI / 4) / (transformStamped2.header.stamp.toSec() - transformStamped1.header.stamp.toSec())));

    ASSERT_EQ(3, poses.size());
    EXPECT_TRUE(targetPoses[0] == poses[0]);
    //pose 1 is not checked becuase we do not know the twist as we do not know when the filter was initialized
    EXPECT_TRUE(targetPoses[2] == poses[2]);
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
    ros::init(argc, argv, "ros_sim_navigation_filter_test");

    broadcastStaticTransform();

    return RUN_ALL_TESTS();
}
