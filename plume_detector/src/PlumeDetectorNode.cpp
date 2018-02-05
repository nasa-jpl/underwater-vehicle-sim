#include "ros/ros.h"

#include "plume_detector/PlumeDetector.h"
#include "plume_detector/DyePlumeDetector.h"

#include "plume_detector/GetPlumeData.h"
std::unique_ptr<PlumeDetector> detector;

bool getPlumeData(plume_detector::GetPlumeData::Request &req,
		     	  plume_detector::GetPlumeData::Response &res)
{
	std::vector<PlumeDetector::PlumeData> plumeData = detector->getPlumeData(req.name, req.start_time, req.end_time);
	

	for(auto dataPoint : plumeData)
	{
		res.x.push_back(dataPoint.x);
		res.y.push_back(dataPoint.y);
		res.h.push_back(dataPoint.h);
		res.time.push_back(dataPoint.time);

		res.val.push_back(dataPoint.val);
	}

	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plume_detector");
    ros::NodeHandle nh("plume_detector");

    detector = std::unique_ptr<PlumeDetector>(new DyePlumeDetector(nh));

    ros::ServiceServer service = nh.advertiseService("get", getPlumeData);

    ros::spin();
}