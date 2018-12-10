#ifndef DATA_BROADCASTER_MODULE_H
#define DATA_BROADCASTER_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

class DataBroadcasterModule : public GeneralModule
{

public:
	DataBroadcasterModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName);
	~DataBroadcasterModule() {}

	void update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState);

	
private:

private:
	ros::Publisher dataRecorder;
	ros::ServiceClient client;
};


#endif
