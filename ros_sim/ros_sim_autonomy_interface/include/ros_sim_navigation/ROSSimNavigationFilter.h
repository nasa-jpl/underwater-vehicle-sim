#ifndef ROS_SIM_NAVIGATION_FILTER
#define ROS_SIM_NAVIGATION_FILTER

#include "ros/ros.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "underwater_navigation/NavigationFilter.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

class ROSSimNavigationFilter
{
public:
    ROSSimNavigationFilter(VehicleInfo info, ros::Publisher posePublisher, std::unique_ptr<NavigationFilter> filter);
    ROSSimNavigationFilter(ROSSimNavigationFilter&& other);
    
    ~ROSSimNavigationFilter() {}

    static ROSSimNavigationFilter createNavigationFilter(std::string& filterName, VehicleInfo info);

    void update();
    void publishPose();

    void sendPoseToFilter();

private:
    tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener;

    VehicleInfo info;
    ros::Publisher posePublisher;
    std::unique_ptr<NavigationFilter> filter;
};

#endif