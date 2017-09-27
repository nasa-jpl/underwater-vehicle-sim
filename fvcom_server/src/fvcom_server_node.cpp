#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fvcom_server");
    ros::NodeHandle n;

    ros::spin();
}