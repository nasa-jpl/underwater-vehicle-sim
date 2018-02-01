#ifndef PLUME_DETECTOR_H
#define PLUME_DETECTOR_H

class PlumeDetector
{
public:

	struct PlumeData
	{
		PlumeData(ros::Time time, double x, double y, double h, double val) :
			time(time),
			x(x),
			y(y),
			h(h),
			val(val) 
		{}

		ros::Time time;
		double x;
		double y;
		double h;
		double val;
	};

	PlumeDetector() {}
	virtual ~PlumeDetector() {}

	virtual std::vector<PlumeData> getPlumeData(std::string vehicleName, ros::Time startTime, ros::Time endTime)=0;
};
#endif