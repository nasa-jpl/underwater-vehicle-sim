#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/VehicleState.h"
class GeneralModule
{
public:
	GeneralModule(std::string name, std::string type, ros::NodeHandle parentNH, std::string vehicleName);

	virtual ~GeneralModule(){}

	virtual void update(std::string name, const ros::Time& lastTime, VehicleState& vehicleState)=0;

	void updateAtRate(std::string name, const ros::Time& lastTime, VehicleState& vehicleState);

	static std::unique_ptr<GeneralModule> makeGeneralModule(std::string moduleName, 
                        ros::NodeHandle& parentNH, std::string vehicleName);

	std::string& getName();
	std::string& getType();
protected:

	std::string name;
	std::string type;
    std::string vehicleName;

	ros::NodeHandle nh;

	ros::Time lastUpdate;
	bool useHertz;
	float hertz;
};


#endif
