#ifndef DYE_PLUME_DETECTOR_H
#define DYE_PLUME_DETECTOR_H

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/PlumeDataEntry.h"

class DyePlumeDetector : public PlumeDetector
{
public:

	DyePlumeDetector();
    
    DyePlumeDetector(DyePlumeDetector&& other);

	virtual ~DyePlumeDetector() {}

	PlumeData getLastPlumeData(std::string vehicleName, DataServer dataServer);
	std::vector<PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer dataServer);

private:

};

#endif