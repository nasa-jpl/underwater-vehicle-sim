#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/VehicleState.h"

#include "model_interface/ModelData.h"

class GeneralModule
{
public:
	GeneralModule(std::string name, std::string type);

	virtual ~GeneralModule(){}

	virtual void update(const ros::Time& lastTime, VehicleState& vehicleState, ModelData& modelData)=0;

	static std::unique_ptr<GeneralModule> makeGeneralModule(std::string moduleName);

	std::string& getName();
	std::string& getType();
protected:

	std::string name;
	std::string type;
    std::string vehicleName;

	ros::NodeHandle nh;

	ros::Time lastUpdate;
};


#endif
