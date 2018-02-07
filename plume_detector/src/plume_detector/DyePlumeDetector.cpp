#include "ros/ros.h"

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/DyePlumeDetector.h"

#include "data_server/GetData.h"

DyePlumeDetector::DyePlumeDetector(ros::NodeHandle handle) :
	nh(nh),
	client(nh.serviceClient<data_server::GetData>("/data_server/get"))
{
}

DyePlumeDetector::DyePlumeDetector(DyePlumeDetector&& other) :
	nh(other.nh),
	client(std::move(other.client))
{}

std::vector<PlumeDetector::PlumeData> DyePlumeDetector::getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime)
{
	std::vector<PlumeData> data;

	data_server::GetData retrievedData;
	retrievedData.request.name = vehicleName;
	retrievedData.request.start_time = startTime;
	retrievedData.request.end_time = endTime;
	client.call(retrievedData);

	for(unsigned int i = 0; i < retrievedData.response.time.size(); i++)
	{
		data.emplace_back(retrievedData.response.time[i],
						  retrievedData.response.x[i],
						  retrievedData.response.y[i],
						  retrievedData.response.h[i],
						  retrievedData.response.dye[i]);
	}
	
	return data;
}