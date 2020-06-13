#include "ros_sim_navigation/ROSSimNavigationFilter.h"
#include "ROSSimVehicleInterface.h"

#include "underwater_autonomy/navigation/TrueNavigationFilter.h"
#include "underwater_autonomy/navigation/DeadReckoningNavigationFilter.h"

#include "underwater_autonomy/sensor_data_types/CommonDataTypes.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

using namespace underwater_autonomy;

ROSSimNavigationFilter::ROSSimNavigationFilter(std::string filterName, std::shared_ptr<ROSSimVehicleInterface> interface) :
    interface(interface),
    filterName(filterName),
    listener(buffer)
{
    initializeNavFilter(filterName);

    ros::NodeHandle filterNh("nav_filters");
    posePublisher = filterNh.advertise<nav_msgs::Odometry>(filterName + "/pose", 10);
    statePublisher = filterNh.advertise<std_msgs::Float64MultiArray>(filterName + "/state", 10);
    covariancePublisher = filterNh.advertise<std_msgs::Float64MultiArray>(filterName + "/covariance", 10);
}

ROSSimNavigationFilter::ROSSimNavigationFilter(ROSSimNavigationFilter&& other) :
    interface(std::move(other.interface)),
    filterName(std::move(other.filterName)),
    listener(buffer),
    posePublisher(std::move(other.posePublisher)),
    statePublisher(std::move(other.statePublisher)),
    covariancePublisher(std::move(other.covariancePublisher)),
    filter(std::move(other.filter))
{}

void ROSSimNavigationFilter::initializeNavFilter(std::string& filterName)
{
    VehicleInfo info = interface->getVehicleInfo();

    ros::NodeHandle filterNhPriv("~/filter/" + filterName);
    ros::NodeHandle filterNh("nav_filters");

    std::string filterType;
    if(!filterNhPriv.getParam("type", filterType))
    {
        ROS_INFO("Parameter \"%s/type\" not present in the parameter server.", filterNhPriv.getNamespace().c_str());
        exit(1);
    }

    VehiclePose startPose(Eigen::Vector3d(info.getStartX(), info.getStartY(), info.getStartZ()));
    if(filterType == "TrueNavigation") {
        filter.reset(new TrueNavigationFilter(interface));
    }
    else if(filterType == "DeadReckoning") {
        filter.reset(new DeadReckoningNavigationFilter(interface));
    }
    filter->setPose(startPose);

}

std::vector<std::vector<double>> ROSSimNavigationFilter::get2dArrayParam(ros::NodeHandle nh, std::string name, std::vector<std::vector<double>> defaultVal)
{
    std::vector<std::vector<double>> returnList;

    XmlRpc::XmlRpcValue list;
    if(nh.getParam(name, list))
    {
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for(int i = 0; i < list.size(); i++)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            returnList.push_back({});
            for(int j = 0; j < list[i].size(); j++)
            {
                ROS_ASSERT(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                           list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
                if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    returnList[i].push_back(static_cast<double>(list[i][j]));
                }
                else if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    returnList[i].push_back(static_cast<int>(list[i][j]));
                }
            }
        }
    }
    else
    {
        returnList = defaultVal;
    
    }
    return returnList;
}

void ROSSimNavigationFilter::update()
{
    filter->update();
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
    odoMsg.child_frame_id = "world_ned"; //twist frame
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

void ROSSimNavigationFilter::publishState()
{
    std::vector<double> state = filter->getState();
    std_msgs::Float64MultiArray msg;
    std_msgs::MultiArrayDimension dim0;

    msg.data.resize(state.size());
    msg.data = state;

    dim0.label = "dim0";
    dim0.size = state.size();
    dim0.stride = state.size();
    msg.layout.dim.push_back(dim0);
    msg.layout.data_offset = 0;

    statePublisher.publish(msg);
}

void ROSSimNavigationFilter::publishStateCovariance()
{
    std::vector<std::vector<double>> covar = filter->getCovariance();
    std_msgs::Float64MultiArray msg;
    std_msgs::MultiArrayDimension dim0;
    std_msgs::MultiArrayDimension dim1;

    if(covar.size() > 0) {
        
        std::vector<double> flattenedCovar;
        for(std::vector<double> a : covar) {
            flattenedCovar.insert(flattenedCovar.end(), a.begin(), a.end());
        }

        msg.data.resize(flattenedCovar.size());
        msg.data = flattenedCovar;

        dim0.label = "dim0";
        dim0.size = covar.size();
        dim0.stride = covar.size() * covar.size();
        dim1.label = "dim1";
        dim1.size = covar.size();
        dim1.stride = covar.size();

        msg.layout.dim.push_back(dim0);
        msg.layout.dim.push_back(dim1);
        msg.layout.data_offset = 0;

        covariancePublisher.publish(msg);
    }
}