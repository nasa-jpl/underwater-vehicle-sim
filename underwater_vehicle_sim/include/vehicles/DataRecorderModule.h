#ifndef DATA_RECORDER_MODULE_H
#define DATA_RECORDER_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

class DataRecorderModule : public GeneralModule
{

public:
	DataRecorderModule(std::string name, ros::NodeHandle& parentNH);
	~DataRecorderModule() {}

	void update(std::string name, const ros::Time& lastTime, const tf::Vector3& position);


	
private:
	/**
	*Callback for the velocity message which is used to control this module
	*@param vel Twist message used to control this module
	*/
	void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

private:
	ros::Publisher dataRecorder;
	ros::ServiceClient client;
};


#endif
