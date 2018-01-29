#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    float loopHertz;
    if(!nh.getParam("planner_node/hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"planner_node/hertz\" not present in the parameter server.");
        exit(1);
    }

    ros::Rate r(loopHertz); //Hz at which to run the update loop

    //Run the update loop
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}