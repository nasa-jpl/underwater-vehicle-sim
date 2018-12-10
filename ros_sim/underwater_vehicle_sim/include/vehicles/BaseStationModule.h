#ifndef BASE_STATION_MODULE_H
#define BASE_STATION_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

class BaseStationModule : public GeneralModule
{

public:
	BaseStationModule(std::string name, ros::NodeHandle& parentNH, std::string vehicleName);
	~BaseStationModule() {}

	void update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState);
	double distanceToBase(const tf::Vector3& position);

	
private:
	/**
	*Callback for the velocity message which is used to control this module
	*@param vel Twist message used to control this module
	*/
	void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

	ros::Publisher pub;
	double base_x;
	double base_y;
	double base_z;
    double base_range;
};


#endif
