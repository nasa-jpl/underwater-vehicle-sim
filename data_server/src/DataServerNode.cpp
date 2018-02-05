#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"
#include "data_server/GetData.h"
#include "std_msgs/String.h"
#include "underwater_vehicle_sim/VehicleData.h"
#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"

#include "ros/ros.h"

DataServer server;

void recieveData(const underwater_vehicle_sim::VehicleData::ConstPtr& msg)
{
	DataServerEntry entry;

	entry.x = msg->x;
	entry.y = msg->y;
	entry.h = msg->h;
	entry.time = msg->time;

	entry.temp = msg->temp;
	entry.salt = msg->salt;
	entry.dye = msg->dye;

	server.putData(msg->name, entry);
}

bool getLatestData(data_server::GetLatestData::Request &req,
			       data_server::GetLatestData::Response &res)
{
	if(server.size(req.name) <= 0)
	{
		return false;
	}

	const DataServerEntry& entry = server.getLatestData(req.name);

	res.x = entry.x;
	res.y = entry.y;
	res.h = entry.h;
	res.time = entry.time;

	res.temp = entry.temp;
	res.dye = entry.dye;
	res.salt = entry.salt;

	return true;
}

bool getData(data_server::GetData::Request &req,
		     data_server::GetData::Response &res)
{
	std::vector<DataServerEntry>::iterator start = server.getStartTime(req.name, req.start_time);
	std::vector<DataServerEntry>::iterator end = server.getEndTime(req.name, req.end_time);

	for(auto it = start; it != end; it++)
	{
		res.x.push_back(it->x);
		res.y.push_back(it->y);
		res.h.push_back(it->h);
		res.time.push_back(it->time);

		res.temp.push_back(it->temp);
		res.dye.push_back(it->dye);
		res.salt.push_back(it->salt);
	}

	return true;
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
    ros::ServiceServer serviceGet = nh.advertiseService("get", getData);
    ros::ServiceServer serviceGetLatest = nh.advertiseService("get_latestatest", getLatestData);

    ros::spin();
}