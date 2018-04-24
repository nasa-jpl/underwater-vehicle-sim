#include "ros/ros.h"

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/DyePlumeDetector.h"

#include "plume_detector/PlumeDataEntry.h"
#include "data_server/GetData.h"

#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"

DyePlumeDetector::DyePlumeDetector() {}

DyePlumeDetector::DyePlumeDetector(DyePlumeDetector&& other) {}

PlumeData DyePlumeDetector::getLastPlumeData(std::string vehicleName, DataServer dataServer)
{
    DataServerEntry entry = dataServer.getLatestData(vehicleName);
    PlumeData data;
    data.time = entry.time;
    data.x = entry.x;
    data.y = entry.y;
    data.h = entry.h;
    data.val = entry.dye;

    return data;
}

std::vector<PlumeData> DyePlumeDetector::getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer dataServer)
{
	std::vector<PlumeData> plumeData;
    std::vector<DataServerEntry>::iterator start = dataServer.getStartTime(vehicleName, startTime);
    std::vector<DataServerEntry>::iterator end = dataServer.getStartTime(vehicleName, endTime);

    for(auto it = start; it != end; it++)
    {
        plumeData.emplace_back(it->time,
                          it->x,
                          it->y,
                          it->h,
                          it->dye);
    }

	return plumeData;
}