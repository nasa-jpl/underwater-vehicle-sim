#include "ros/ros.h"

#include "data_server/DataServer.h"
#include "underwater_vehicle_sim/VehicleData.h"

DataServer server;

bool recieveData(const underwater_vehicle_sim::VehicleData::ConstPtr& msg)
{    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_server");
    ros::NodeHandle n;

    ros::spin();
}