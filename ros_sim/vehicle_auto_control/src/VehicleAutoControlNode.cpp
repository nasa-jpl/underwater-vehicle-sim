#include "ros/ros.h"

#include "vehicle_auto_control/VehicleController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_auto_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    float loopHertz;
    if(!nhPriv.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    VehicleController controller;

    ros::Rate loop(loopHertz);
    while(ros::ok())
    {
        controller.update();
        ros::spinOnce();
        loop.sleep();
    }
        return 0;
}