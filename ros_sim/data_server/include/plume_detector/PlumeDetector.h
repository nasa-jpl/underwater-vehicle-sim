#ifndef PLUME_DETECTOR_H
#define PLUME_DETECTOR_H

#include "underwater_vehicle_msgs/VehicleData.h"
#include "plume_detector/PlumeDataEntry.h"
#include "data_server/DataServer.h"

class PlumeDetector
{
public:

	PlumeDetector() {}
	virtual ~PlumeDetector() {}

	virtual float calcPlumeStrength(std::string name, const underwater_vehicle_msgs::VehicleData::ConstPtr& newData, DataServer& dataServer)=0;
	virtual std::vector<PlumeDataEntry> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer& dataServer)=0;
};
#endif