#include "ros/ros.h"

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/DyePlumeDetector.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "plume_detector/PlumeDataEntry.h"
#include "data_server/GetData.h"

#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"

DyePlumeDetector::DyePlumeDetector() {}

DyePlumeDetector::DyePlumeDetector(DyePlumeDetector&& other) {}

float DyePlumeDetector::calcPlumeStrength(std::string name, const underwater_vehicle_sim::VehicleData::ConstPtr& newData, DataServer dataServer)
{
    return newData->dye;
}

std::vector<PlumeDataEntry> DyePlumeDetector::getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer dataServer)
{
	std::vector<PlumeDataEntry> plumeData;
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