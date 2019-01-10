#include <vector>
#include <memory>

#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"
#include "data_server/GetData.h"
#include "data_server/SaveData.h"
#include "std_msgs/String.h"
#include "underwater_vehicle_msgs/VehicleData.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "data_server/GetData.h"
#include "data_server/ClearData.h"
#include "data_server/GetLatestData.h"
#include "data_server/GetPlumeData.h"
#include "data_server/PlumeData.h"

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/DyePlumeDetector.h"
#include "plume_detector/PlumeDataEntry.h"
#include "ros/ros.h"

DataServer server;
std::unique_ptr<PlumeDetector> plumeDetector;
std::map<std::string, ros::Publisher> plumePubs;

void recieveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg)
{
    float plumeVal = plumeDetector->calcPlumeStrength(msg->name, msg, server);

	DataServerEntry entry;

	entry.x = msg->x;
	entry.y = msg->y;
	entry.h = msg->h;
	entry.time = msg->time;

	entry.temp = msg->temp;
	entry.salt = msg->salt;
	entry.dye = msg->dye;
	entry.sonarDepth = msg->sonarDepth;
    entry.plumeStrength = plumeVal;

	server.putData(msg->name, entry);


    data_server::PlumeData plumeMsg;
    plumeMsg.x = entry.x;
    plumeMsg.y = entry.y;
    plumeMsg.h = entry.h;
    plumeMsg.time = entry.time;
    plumeMsg.plume_strength = entry.plumeStrength;

    plumePubs[msg->name].publish(plumeMsg);
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
    res.sonarDepth = entry.sonarDepth;

	return true;
}

bool getData(data_server::GetData::Request &req,
		     data_server::GetData::Response &res)
{
	if(server.size(req.name) == 0)
	{
		return true;
	}

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
		res.sonarDepth.push_back(it->sonarDepth);
	}

	return true;
}

bool clearData(data_server::ClearData::Request &req,
               data_server::ClearData::Response &res)
{
    server.clear(req.name);
    return true;
}

bool getPlumeData(data_server::GetPlumeData::Request &req,
                  data_server::GetPlumeData::Response &res)
{
    std::vector<PlumeDataEntry> plumeData = plumeDetector->getPlumeData(req.name, req.start_time, req.end_time, server);

    for(PlumeDataEntry& data : plumeData)
    {
        res.x.push_back(data.x);
        res.y.push_back(data.y);
        res.h.push_back(data.h);
        res.time.push_back(data.time);
        res.plume_val.push_back(data.val);
    }

    return true;
}

bool saveData(data_server::SaveData::Request &req,
			  data_server::SaveData::Response &res)
{
	ROS_INFO("Start Saving Data");
	server.saveToFile(req.filename);
	ROS_INFO("Finished Saving Data");

	res.success = true;
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_server");
    ros::NodeHandle nh("data_server");

    std::vector<std::string> vehicleNames;
    nh.getParam("/underwater_vehicle_sim/vehicles/names", vehicleNames);
	ros::ServiceClient infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("/underwater_vehicle_sim/vehicles/get_info");
	infoClient.waitForExistence();

	std::vector<ros::Subscriber> subscribers;

    plumeDetector = std::unique_ptr<PlumeDetector>(new DyePlumeDetector());

	for(std::string& name : vehicleNames)
	{
		underwater_vehicle_msgs::GetVehicleInfo info;
		info.request.name = name;
		infoClient.call(info);
		
        for(unsigned i = 0; i < info.response.moduleNames.size(); i++)
		{

			if(info.response.moduleTypes[i] == "DataBroadcaster")
			{
				subscribers.push_back(nh.subscribe("/underwater_vehicle_sim/vehicles/" + name + "/" + info.response.moduleNames[i] + "/data", 5000, recieveData));
			}
		}

        plumePubs.insert(std::pair<std::string, ros::Publisher>(
                    name,
                    nh.advertise<data_server::PlumeData>(name + "/plume_data", 1000)));
	}

    ros::ServiceServer saveDataSub = nh.advertiseService("save", saveData);
    ros::ServiceServer serviceGet = nh.advertiseService("get", getData);
    ros::ServiceServer serviceClear = nh.advertiseService("clear", clearData);
    ros::ServiceServer serviceGetPlume = nh.advertiseService("get_plume", getPlumeData);
    ros::ServiceServer serviceGetLatest = nh.advertiseService("get_latest", getLatestData);

    ros::spin();
}