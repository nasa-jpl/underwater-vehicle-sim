#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_navigation");
    ros::NodeHandle nh;

    //Wait until the simulation starts to proceed
    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
    vehicleInfoClient.waitForExistence();


    float loopHertz;
    if(!nh.getParam("navigation/hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"navigation/hertz\" not present in the parameter server.");
        exit(1);
    }

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}