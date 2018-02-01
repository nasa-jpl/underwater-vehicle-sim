#ifndef DYE_PLUME_DETECTOR_H
#define DYE_PLUME_DETECTOR_H

#include "plume_detector/PlumeDetector.h"

class DyePlumeDetector : public PlumeDetector
{
public:

	DyePlumeDetector(ros::NodeHandle& handle);
	virtual ~DyePlumeDetector() {}

	std::vector<PlumeDetector::PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime);

private:

	ros::NodeHandle& nh;
	ros::ServiceClient client;
};
#endif