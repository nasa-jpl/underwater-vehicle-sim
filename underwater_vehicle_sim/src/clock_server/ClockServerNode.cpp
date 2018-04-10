#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Float64.h"
#include <chrono>
#include <iostream>
#include <thread>

float speedUpFactor, simStartTime;

void getSpeedUpFactor(const std_msgs::Float64 factor)
{
    speedUpFactor = fabs(factor.data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clock_server");
    ros::NodeHandle n;

    ros::Publisher clockPub = n.advertise<rosgraph_msgs::Clock>("/clock", 1000);
    ros::Subscriber sub = n.subscribe("clock_server/speed_up_factor", 1, &getSpeedUpFactor);

    ros::WallDuration interval = ros::WallDuration(0.001);

    
	

	n.param<float>("speed_up_factor", speedUpFactor, 1);
	n.param<float>("sim_start_time", simStartTime, 0);

    //Tracks the previous time published to prevent going back in time
    ros::Time prevRosTime = ros::Time(simStartTime);
    
    ros::WallTime prev = ros::WallTime::now();
    while(ros::ok())
    {
    	//get clock time
        auto now = ros::WallTime::now();

        //get time from system
        ros::WallDuration chronoTime = (now - prev) * speedUpFactor;

        //convert system time to ros time
        ros::Time rosTime(chronoTime.toSec() + prevRosTime.toSec());

        //Create ROS message for clock topic
        rosgraph_msgs::Clock msg;
        msg.clock = rosTime;

        if(rosTime > prevRosTime)
        {
            //publish message
            clockPub.publish(msg);
            prevRosTime = rosTime;
            prev = now;
        }
        

        ros::spinOnce();

        // delay until time to iterate again
        interval.sleep();
    }

}
