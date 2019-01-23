#include "ros_sim_navigation/ROSSimNavigationFilter.h"

#include "underwater_navigation/TrueNavigationFilter.h"

#include "nav_msgs/Odometry.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

ROSSimNavigationFilter::ROSSimNavigationFilter(VehicleInfo info, ros::Publisher posePublisher, std::unique_ptr<NavigationFilter> filter) :
    info(info),
    posePublisher(std::move(posePublisher)),
    filter(std::move(filter)),
    listener(buffer)
{}

ROSSimNavigationFilter::ROSSimNavigationFilter(ROSSimNavigationFilter&& other) :
    info(std::move(other.info)),
    posePublisher(std::move(other.posePublisher)),
    filter(std::move(other.filter)),
    listener(buffer)
{}

ROSSimNavigationFilter ROSSimNavigationFilter::createNavigationFilter(ros::NodeHandle& nhRoot, ros::NodeHandle& nhNav, std::string& filterName, VehicleInfo info)
{
    ros::NodeHandle filterNH(nhNav, "filter/" + filterName);

    std::string filterType;
    if(!filterNH.getParam("type", filterType))
    {
        ROS_INFO("Parameter \"%s/type\" not present in the parameter server.", filterNH.getNamespace().c_str());
        exit(1);
    }

    ros::Publisher posePublisher = nhNav.advertise<nav_msgs::Odometry>(info.getName() + "/" + filterName, 1);
    std::unique_ptr<NavigationFilter> filter;

    if(filterType == "TrueNavigation")
    {
        VehiclePose startPose(Eigen::Vector3d(info.getStartX(), info.getStartY(), info.getStartZ()));
        filter.reset(new TrueNavigationFilter(startPose, ros::Time::now().toSec()));
    }

    return ROSSimNavigationFilter(info, std::move(posePublisher), std::move(filter));
}

void ROSSimNavigationFilter::update()
{
    sendPoseToFilter();
}

void ROSSimNavigationFilter::publishPose()
{
    VehiclePose pose = filter->getPoseEstimation();

    Eigen::Vector3d position = pose.getPosition();
    Eigen::Quaterniond orientation = pose.getOrientation();
    Eigen::Matrix<double,6,6> poseCovariance = pose.getPoseCovariance();

    Eigen::Vector3d linearVelocity = pose.getLinearVelocity();
    Eigen::Vector3d angularVelocity = pose.getAngularVelocity();
    Eigen::Matrix<double,6,6> twistCovariance = pose.getTwistCovariance();

    nav_msgs::Odometry odoMsg;
    odoMsg.header.stamp = ros::Time::now();
    odoMsg.header.frame_id = "world_ned"; //pose frame
    odoMsg.child_frame_id = info.getName(); //twist frame
    odoMsg.pose.pose.position.x = position[0];
    odoMsg.pose.pose.position.y = position[1];
    odoMsg.pose.pose.position.z = position[2];

    odoMsg.pose.pose.orientation.x = orientation.x();
    odoMsg.pose.pose.orientation.y = orientation.y();
    odoMsg.pose.pose.orientation.z = orientation.z();
    odoMsg.pose.pose.orientation.w = orientation.w();

    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            odoMsg.pose.covariance[(i * 6) + j] = poseCovariance(i, j);
        }
    }

    odoMsg.twist.twist.linear.x = linearVelocity[0];
    odoMsg.twist.twist.linear.y = linearVelocity[1];
    odoMsg.twist.twist.linear.z = linearVelocity[2];

    odoMsg.twist.twist.angular.x = angularVelocity[0];
    odoMsg.twist.twist.angular.y = angularVelocity[1];
    odoMsg.twist.twist.angular.z = angularVelocity[2];

    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            odoMsg.twist.covariance[(i * 6) + j] = twistCovariance(i, j);
        }
    }

    posePublisher.publish(odoMsg);
}

void ROSSimNavigationFilter::sendPoseToFilter()
{
    geometry_msgs::TransformStamped transformMsg;
	try
    {
        if(buffer.canTransform("world_ned", info.getName(), ros::Time(0), ros::Duration(1.0)))
        {
            transformMsg = buffer.lookupTransform("world_ned", info.getName(), ros::Time(0));
            std::vector<double> data;
            data.push_back(transformMsg.transform.translation.x); //x position
            data.push_back(transformMsg.transform.translation.y); //y position
            data.push_back(transformMsg.transform.translation.z); //z position
            data.push_back(transformMsg.transform.rotation.x); //x orientation
            data.push_back(transformMsg.transform.rotation.y); //y orientation
            data.push_back(transformMsg.transform.rotation.z); //z orientation
            data.push_back(transformMsg.transform.rotation.w); //w orientation

            filter->sensorMeasurment("pose", transformMsg.header.stamp.toSec(), data);
        }

        
	}
	catch(tf2::TransformException ex)
	{
		throw ex;
	}   
}