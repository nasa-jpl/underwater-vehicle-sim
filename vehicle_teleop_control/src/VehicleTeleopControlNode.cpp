#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void addAllPublishers(std::vector<ros::Publisher>& publishers, ros::NodeHandle& nh)
{
    //Create the vehicle objects
    std::vector<std::string> vehicleNames;
    nh.getParam("vehicles/names", vehicleNames);

    for(std::string& name : vehicleNames)
    {
        std::string propModuleName;
        std::string topic;

        //get the name of the propulsion module and create the needed 

        if(nh.hasParam("vehicles/" + name + "/propModuleName"))
        {
            nh.getParam("vehicles/" + name + "/propModuleName", propModuleName);
            topic = "vehicles/" + name + "/" + propModuleName + "/command_velocity";
            ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic, 1);
            publishers.push_back(pub);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_teleop_control");
    ros::NodeHandle nh;

    std::vector<ros::Publisher> publishers;
    addAllPublishers(publishers, nh);


    signal(SIGINT,quit);

    unsigned int currentVehicle;

    double linearX, linearY, linearZ, rotateX, rotateY, rotateZ;

    char c;
    bool dirty = false;


    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);

    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use keys to move the vehicle.");

    //Run the input loop
    while(true)
    {
        // get the next event from the keyboard  
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linearX = linearY = linearZ = 0;
        rotateX = rotateY = rotateZ = 0;

        ROS_DEBUG("value: 0x%02X\n", c);
      
        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                rotateZ = -1.0;
                dirty = true;
                break;

            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                rotateZ = 1.0;
                dirty = true;
                break;

            case KEYCODE_U:
                ROS_DEBUG("UP");
                linearX = 1.0;
                dirty = true;
                break;

            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                linearX = -1.0;
                dirty = true;
                break;
        }
       

        geometry_msgs::Twist twist;
        twist.angular.x = rotateX;
        twist.angular.y = rotateY;
        twist.angular.z = rotateZ;
        twist.linear.x = linearX;
        twist.linear.y = linearY;
        twist.linear.z = linearZ;

        if(dirty && currentVehicle < publishers.size())
        {
            publishers[currentVehicle].publish(twist);    
            dirty = false;
        }
    }
    return(0);
}
