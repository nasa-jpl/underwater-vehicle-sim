#include "ros/ros.h"

#include "data_server/DataServer.h"


DataServer server;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_server");
    ros::NodeHandle n;

    ros::spin();
}
