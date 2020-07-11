#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "tf2_ros/transform_listener.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "underwater_autonomy/navigation/single_beacon_filter/SingleBeaconNavigationFilter.h"

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

using namespace underwater_autonomy;

ros::Publisher beaconPublisher;
ros::Publisher optResultPublisher;
Vector2dData lastBeaconPosition;
Vector2dData lastOptResult;

std::unique_ptr<SingleBeaconNavigationFilter> filter;
ros::Timer updateTimer;

void publishBeaconPosition() {
    Vector2dData newOptResult = filter->getLastOptimizationResults();
    Vector2dData newBeaconPosition = filter->getBeaconEstimateInVehicleFrame();

    if(newOptResult.time != lastOptResult.time) {
        geometry_msgs::Point msg;
        msg.x = newOptResult.data[0];
        msg.y = newOptResult.data[1];
        optResultPublisher.publish(msg);

        lastOptResult = newOptResult;
    }

    if(newBeaconPosition.time != lastBeaconPosition.time) {
        geometry_msgs::Point msg;
        msg.x = newBeaconPosition.data[0];
        msg.y = newBeaconPosition.data[1];
        beaconPublisher.publish(msg);
        
        lastBeaconPosition = newBeaconPosition;
    }
}

void timerCallback(const ros::TimerEvent&) {
    filter->update();
    publishBeaconPosition();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_dr_navigation");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    vehicleInfoClient.waitForExistence();
    underwater_vehicle_msgs::GetVehicleInfo getInfo;
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);
    std::shared_ptr<ROSSimVehicleInterface> interface = std::make_shared<ROSSimVehicleInterface>(info);

    float hertz;
    if(!nhPriv.getParam("hertz", hertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    std::string configFilename;
    nhPriv.getParam("config_file", configFilename);
    ConfigurationFile config(configFilename);
    SingleBeaconNavigationFilter::Parameters parameters(config);

    filter = std::unique_ptr<SingleBeaconNavigationFilter>(new SingleBeaconNavigationFilter(interface, parameters));

    beaconPublisher = nh.advertise<geometry_msgs::Point>("single_beacon_nav/beacon", 10);
    optResultPublisher = nh.advertise<geometry_msgs::Point>("single_beacon_nav/optimization_result", 10);

    updateTimer = nh.createTimer(ros::Duration(1/hertz), timerCallback);
    ros::spin();
}