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

    auto interval = std::chrono::milliseconds(10);
	auto nodeStart = std::chrono::steady_clock::now();


	

	n.param<float>("speed_up_factor", speedUpFactor, 1);
	n.param<float>("sim_start_time", simStartTime, 0);

    while(ros::ok())
    {
    	//get clock time
        auto now = std::chrono::steady_clock::now();

        //get time from system
        std::chrono::duration<double> chronoTime = (now - nodeStart) * speedUpFactor;

        //convert system time to ros time
        ros::Time rosTime(chronoTime.count() + simStartTime);

        //Create ROS message for clock topic
        rosgraph_msgs::Clock msg;
        msg.clock = rosTime;

        //publish message
        clockPub.publish(msg);

        ros::spinOnce();

        // delay until time to iterate again
        auto next = now + interval;
        std::this_thread::sleep_until(next);
    }

}
