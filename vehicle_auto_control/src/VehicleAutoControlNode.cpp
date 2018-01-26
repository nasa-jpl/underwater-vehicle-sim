#include "ros/ros.h"

#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/PointPath.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_auto_control");
    ros::NodeHandle nh;

    float loopHertz;
    if(!nh.getParam("vehicle_controller/hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"vehicle_controller/hertz\" not present in the parameter server.");
        exit(1);
    }

    VehicleController controller(nh);

    ros::Rate r(loopHertz); //Hz at which to run the update loop

    //Run the update loop
    while(ros::ok())
    {
        controller.update();

        ros::spinOnce();
        r.sleep();
    }
}