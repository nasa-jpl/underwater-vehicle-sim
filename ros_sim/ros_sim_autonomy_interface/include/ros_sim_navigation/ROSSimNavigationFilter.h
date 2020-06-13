#ifndef ROS_SIM_NAVIGATION_FILTER
#define ROS_SIM_NAVIGATION_FILTER

#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_listener.h"

#include "underwater_autonomy/navigation/NavigationFilter.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ROSSimVehicleInterface.h"

class ROSSimNavigationFilter
{
public:

    ROSSimNavigationFilter(std::string filterName, std::shared_ptr<ROSSimVehicleInterface> interface);
    ROSSimNavigationFilter(ROSSimNavigationFilter&& other);
    
    ~ROSSimNavigationFilter() {}

    void update();
    void publishPose();
    void publishState();
    void publishStateCovariance();

private:
    void initializeNavFilter(std::string& filterName);
    static std::vector<std::vector<double>> get2dArrayParam(ros::NodeHandle nh, std::string name, std::vector<std::vector<double>> defaultVal);
private:
    std::shared_ptr<ROSSimVehicleInterface> interface;
    std::string filterName;

    tf2_ros::Buffer buffer;
 	tf2_ros::TransformListener listener;

    
    ros::Publisher posePublisher;
    ros::Publisher statePublisher;
    ros::Publisher covariancePublisher;

    std::unique_ptr<underwater_autonomy::NavigationFilter> filter;
};

#endif