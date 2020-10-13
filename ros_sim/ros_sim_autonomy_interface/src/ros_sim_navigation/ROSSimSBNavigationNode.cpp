#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include "underwater_vehicle_msgs/Ranges.h"
#include "underwater_vehicle_msgs/Range.h"

#include "tf2_ros/transform_listener.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "underwater_autonomy/navigation/single_beacon_filter/SingleBeaconNavigationFilter.h"
#include "underwater_autonomy/navigation/single_beacon_filter/SyntheticMultiLaterationInterface.h"

#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

using namespace underwater_autonomy;

ros::Publisher beaconPublisher;
ros::Publisher rangePublisher;
ros::Publisher optResultPublisher;
Vector2dData lastBeaconPosition;
Vector4dData lastOptResult;

std::unique_ptr<SingleBeaconNavigationFilter> filter;
ros::Timer updateTimer;

void publishBeaconPosition() {
    Vector4dData newOptResult = filter->getLastOptimizationResults();
    Vector2dData newBeaconPosition = filter->getBeaconEstimateInVehicleFrame();

    if(newOptResult.time != lastOptResult.time) {
        std_msgs::Float64MultiArray msg;
        std::vector<double> msgArray;
        msgArray.push_back(newOptResult.data[0]);
        msgArray.push_back(newOptResult.data[1]);
        msgArray.push_back(newOptResult.data[2]);
        msgArray.push_back(newOptResult.data[3]);
        msg.data = msgArray;
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

void publishRanges() {
    std::vector<MultiLaterationRange> ranges = filter->getRanges();
    underwater_vehicle_msgs::Ranges allRangesMsg;
    std::vector<underwater_vehicle_msgs::Range> msgs;
    for(auto r : ranges) {
        underwater_vehicle_msgs::Range rangeMsg;
        rangeMsg.x = r.position.data[0];
        rangeMsg.y = r.position.data[1];
        rangeMsg.position_covariance[0] = r.position.covariance(0,0);
        rangeMsg.position_covariance[1] = r.position.covariance(0,1);
        rangeMsg.position_covariance[2] = r.position.covariance(1,0);
        rangeMsg.position_covariance[3] = r.position.covariance(1,1);

        rangeMsg.range = r.range.data;
        rangeMsg.range_variance = r.range.variance;
        msgs.push_back(rangeMsg);
    }
    allRangesMsg.ranges = msgs;
    rangePublisher.publish(allRangesMsg);
}

void timerCallback(const ros::TimerEvent&) {
    filter->update();
    publishBeaconPosition();
    publishRanges();
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
    optResultPublisher = nh.advertise<std_msgs::Float64MultiArray>("single_beacon_nav/optimization_result", 10);
    rangePublisher = nh.advertise<underwater_vehicle_msgs::Ranges>("single_beacon_nav/ranges", 10);

    updateTimer = nh.createTimer(ros::Duration(1/hertz), timerCallback);
    ros::spin();
}