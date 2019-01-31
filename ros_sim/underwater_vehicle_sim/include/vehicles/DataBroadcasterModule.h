#ifndef DATA_BROADCASTER_MODULE_H
#define DATA_BROADCASTER_MODULE_H

#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

class DataBroadcasterModule : public GeneralModule
{

public:
	DataBroadcasterModule(std::string name);
	~DataBroadcasterModule() {}

	void update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData);

	
private:

private:
	ros::Publisher dataRecorder;
	ros::ServiceClient client;
};


#endif
