#include "ros/ros.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/PoseWithCovariance.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_navigation/ROSSimNavigationFilter.h"

#include "underwater_navigation/NavigationFilter.h"
#include "underwater_navigation/TrueNavigationFilter.h"

std::vector<ROSSimNavigationFilter> vehicleFilters;
ros::ServiceClient vehicleInfoClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_navigation");

    ros::NodeHandle nhRoot;
    ros::NodeHandle nhNav("navigation");

    vehicleInfoClient = nhRoot.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
    vehicleInfoClient.waitForExistence();

    float loopHertz;
    std::vector<std::string> filterNames;    
    
    if(!nhNav.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhNav.getNamespace().c_str());
        exit(1);
    }
    
    if(!nhNav.getParam("filter/names", filterNames))
    {
        ROS_FATAL("Parameter \"%s/filter/names\" not present in the parameter server.", nhNav.getNamespace().c_str());
        exit(1);
    }
    
    for(std::string& filterName : filterNames)
    {
        ros::NodeHandle nhFilter(nhNav, "filter/" + filterName);

        std::string vehicleName;
        if(!nhFilter.getParam("vehicle_name", vehicleName))
        {
            ROS_INFO("Parameter \"%s/vehicle_name\" not present in the parameter server.", nhFilter.getNamespace().c_str());
            exit(1);
        }

        underwater_vehicle_msgs::GetVehicleInfo getInfo;
        getInfo.request.name = vehicleName;
        vehicleInfoClient.call(getInfo);
        VehicleInfo info(getInfo);

        ROSSimNavigationFilter filter = ROSSimNavigationFilter::createNavigationFilter(nhRoot, nhNav, filterName, info);
        vehicleFilters.push_back(std::move(filter));
    }

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        for(ROSSimNavigationFilter& vehicleFilter : vehicleFilters)
        {
            vehicleFilter.update();
            vehicleFilter.publishPose();
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}