#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");
    ros::NodeHandle n;

    while(ros::ok())
    {
        ros::spinOnce();
    }
}