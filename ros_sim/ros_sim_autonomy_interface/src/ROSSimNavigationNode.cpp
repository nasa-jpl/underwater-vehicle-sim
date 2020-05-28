#include "ros/ros.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/PoseWithCovariance.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_navigation/ROSSimNavigationFilter.h"

#include "underwater_autonomy/navigation/NavigationFilter.h"
#include "underwater_autonomy/navigation/TrueNavigationFilter.h"

std::vector<ROSSimNavigationFilter> vehicleFilters;
ros::ServiceClient vehicleInfoClient;

void timerCallback(const ros::TimerEvent&) {
    for(ROSSimNavigationFilter& vehicleFilter : vehicleFilters)
    {
        vehicleFilter.update();
        vehicleFilter.publishPose();
        vehicleFilter.publishState();
        vehicleFilter.publishStateCovariance();

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_navigation");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    vehicleInfoClient.waitForExistence();

    float loopHertz;
    std::vector<std::string> filterNames;    
    
    if(!nhPriv.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }
    
    if(!nhPriv.getParam("filter_names", filterNames))
    {
        ROS_FATAL("Parameter \"%s/filter_names\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }
    
    for(std::string filterName : filterNames)
    {
        underwater_vehicle_msgs::GetVehicleInfo getInfo;
        vehicleInfoClient.call(getInfo);
        VehicleInfo info(getInfo);

        vehicleFilters.emplace_back(filterName, info);
    }

    ros::Timer timer = nh.createTimer(ros::Duration(1/loopHertz), timerCallback);

    ros::spin();
    return 0;
}