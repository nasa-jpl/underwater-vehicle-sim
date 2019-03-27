#ifndef ROS_SIM_NAVIGATION_FILTER
#define ROS_SIM_NAVIGATION_FILTER

#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "sensor_msgs/Imu.h"
#include "underwater_vehicle_msgs/USBL.h"
#include "underwater_vehicle_msgs/FloatMeasurement.h"

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

    void sendIMUToFilter(sensor_msgs::Imu imuData);
    void sendDepthToFilter(underwater_vehicle_msgs::FloatMeasurement depthData);
    void sendUSBLToFilter(underwater_vehicle_msgs::USBL usblData);

    void sendForwardThruster(underwater_vehicle_msgs::FloatMeasurement forwardData);
    void sendLateralThruster(underwater_vehicle_msgs::FloatMeasurement lateralData);

private:
    tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener;

    VehicleInfo info;
    ros::Publisher posePublisher;
    std::unique_ptr<NavigationFilter> filter;

    ros::Subscriber imuData;
    ros::Subscriber usblData;
    ros::Subscriber depthData;

    ros::Subscriber forwardThrusterData;
    ros::Subscriber lateralThrusterData;
};

#endif