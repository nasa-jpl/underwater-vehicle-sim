#include "ros/ros.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "underwater_navigation/NavigationFilter.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"


struct VehicleFilter
{
    std::string vehicleName;
    ros::Publisher posePublisher;
    std::unique_ptr<NavigationFilter> filter;
};

std::vector<VehicleFilter> vehicleFilters;
ros::ServiceClient vehicleInfoClient;

VehicleFilter createVehicleFilter(ros::NodeHandle nhNav, std::string filterName)
{
    ros::NodeHandle filterNH(nhNav, "filters/" + filterName);
    std::string vehicleName;

    if(!filterNH.getParam("vehicle_name", vehicleName))
    {
        std::string test = "test";
        ROS_INFO("Parameter \"%s/vehicle_name\" not present in the parameter server.", filterNH.getNamespace().c_str());
        exit(1);
    }

    underwater_vehicle_msgs::GetVehicleInfo getInfo;
    getInfo.request.name = vehicleName;
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);

    VehicleFilter filter;
    filter.vehicleName = vehicleName;
    filter.posePublisher = nhNav.advertise<geometry_msgs::PoseWithCovariance>(vehicleName + "/" + filterName, 1);

    return filter;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_navigation");

    ros::NodeHandle nhRoot;
    ros::NodeHandle nhNav("navigation");

    float loopHertz;
    std::vector<std::string> filterNames;    

    //Wait until the simulation starts to proceed
    vehicleInfoClient = nhRoot.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
    vehicleInfoClient.waitForExistence();
    
    if(!nhNav.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhNav.getNamespace().c_str());
        exit(1);
    }
    
    if(!nhNav.getParam("filters", filterNames))
    {
        ROS_FATAL("Parameter \"%s/filters\" not present in the parameter server.", nhNav.getNamespace().c_str());
        exit(1);
    }
    
    for(std::string& filterName : filterNames)
    {
        VehicleFilter filter = createVehicleFilter(nhNav, filterName);
        vehicleFilters.push_back(std::move(filter));
    }

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        for(VehicleFilter& vehicleFilter : vehicleFilters)
        {
            VehiclePose pose = vehicleFilter.filter->getPoseEstimation();

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

            vehicleFilter.posePublisher.publish(poseMsg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}