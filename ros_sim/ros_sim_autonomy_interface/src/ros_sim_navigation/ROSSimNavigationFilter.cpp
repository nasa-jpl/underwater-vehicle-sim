#include "ros_sim_navigation/ROSSimNavigationFilter.h"

#include "underwater_navigation/TrueNavigationFilter.h"

#include "geometry_msgs/PoseWithCovariance.h"
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

    ros::Publisher posePublisher = nhNav.advertise<geometry_msgs::PoseWithCovariance>(info.getName() + "/" + filterName, 1);
    std::unique_ptr<NavigationFilter> filter;

    if(filterType == "TrueNavigation")
    {
        VehiclePose startPose(Eigen::Vector3d(info.getStartX(), info.getStartY(), info.getStartZ()));
        filter.reset(new TrueNavigationFilter(startPose));
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
    Eigen::Matrix<double,6,6> covariance = pose.getCovariance();

    geometry_msgs::PoseWithCovariance poseMsg;
    poseMsg.pose.position.x = position[0];
    poseMsg.pose.position.y = position[1];
    poseMsg.pose.position.z = position[2];

    poseMsg.pose.orientation.x = orientation.x();
    poseMsg.pose.orientation.y = orientation.y();
    poseMsg.pose.orientation.z = orientation.z();
    poseMsg.pose.orientation.w = orientation.w();

    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            poseMsg.covariance[(i * 6) + j] = covariance(i, j);
        }
    }

    posePublisher.publish(poseMsg);
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

            filter->sensorMeasurment("pose", data);
        }

        
	}
	catch(tf2::TransformException ex)
	{
		throw ex;
	}   
}