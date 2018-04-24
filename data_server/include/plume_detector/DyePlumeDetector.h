#ifndef DYE_PLUME_DETECTOR_H
#define DYE_PLUME_DETECTOR_H

#include "underwater_vehicle_sim/VehicleData.h"
#include "plume_detector/PlumeDetector.h"
#include "plume_detector/PlumeDataEntry.h"

class DyePlumeDetector : public PlumeDetector
{
public:

	DyePlumeDetector();
    
    DyePlumeDetector(DyePlumeDetector&& other);

	virtual ~DyePlumeDetector() {}

	float calcPlumeStrength(std::string name, const underwater_vehicle_sim::VehicleData::ConstPtr& newData, DataServer dataServer);
	std::vector<PlumeDataEntry> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime, DataServer dataServer);

private:

};

#endif