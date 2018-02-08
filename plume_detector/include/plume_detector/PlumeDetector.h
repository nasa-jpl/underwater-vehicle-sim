#ifndef PLUME_DETECTOR_H
#define PLUME_DETECTOR_H

#include "plume_detector/PlumeData.h"

class PlumeDetector
{
public:

	

	PlumeDetector() {}
	virtual ~PlumeDetector() {}

	virtual std::vector<PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime)=0;
};
#endif