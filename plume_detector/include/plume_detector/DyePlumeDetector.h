#ifndef DYE_PLUME_DETECTOR_H
#define DYE_PLUME_DETECTOR_H

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/PlumeData.h"

class DyePlumeDetector : public PlumeDetector
{
public:

	DyePlumeDetector(ros::NodeHandle handle);
    
    DyePlumeDetector(DyePlumeDetector&& other);

	virtual ~DyePlumeDetector() {}

	std::vector<PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime);

private:

	ros::NodeHandle nh;
	ros::ServiceClient client;
};

#endif