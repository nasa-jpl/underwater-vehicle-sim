#include "data_server/DataServer.h"
#include "data_server/GetData.h"
#include "std_msgs/String.h"

#include "underwater_vehicle_sim/VehicleData.h"

#include "ros/ros.h"


DataServer server;

void recieveData(const underwater_vehicle_sim::VehicleData::ConstPtr& msg)
{
	DataServer::DataServerEntry entry;

	entry.x = msg->x;
	entry.y = msg->y;
	entry.h = msg->h;
	entry.time = msg->time;

	entry.temp = msg->temp;
	entry.salt = msg->salt;
	entry.dye = msg->dye;

	server.putData(msg->name, entry);
}

void saveData(const std_msgs::String::ConstPtr& msg)
{
	server.saveToFile(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_server");
    ros::NodeHandle nh("data_server");

    ros::Subscriber recievedDataSub = nh.subscribe("put", 5000, recieveData);
    ros::Subscriber saveDataSub = nh.subscribe("save", 5000, saveData);
    ros::spin();
}