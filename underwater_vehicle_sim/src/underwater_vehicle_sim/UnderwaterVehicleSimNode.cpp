#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");
    ros::NodeHandle n;

    ros::Rate r(100); //Hz at which to run the sim loop

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}