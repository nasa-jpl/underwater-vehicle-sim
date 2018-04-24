#ifndef PLUME_DETECTOR_H
#define PLUME_DETECTOR_H

#include "plume_detector/PlumeDataEntry.h"
#include "data_server/DataServer.h"

class PlumeDetector
{
public:

	PlumeDetector() {}
	virtual ~PlumeDetector() {}

	virtual PlumeData getLastPlumeData(std::string vehicleName, DataServer dataServer)=0;
	virtual std::vector<PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer dataServer)=0;
};
#endif