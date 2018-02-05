#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

class GeneralModule
{
public:
	GeneralModule(std::string name, std::string type, ros::NodeHandle parentNH);

	virtual ~GeneralModule(){}

	virtual void update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, 
						double& powerCapacity, double& dataCapacity)=0;

	void updateAtRate(std::string name, const ros::Time& lastTime, const tf::Vector3& position,
						double& powerCapacity, double& dataCapacity);

	static std::unique_ptr<GeneralModule> makeGeneralModule(std::string moduleName, ros::NodeHandle& parentNH);

	std::string& getName();
	std::string& getType();
protected:

	std::string name;
	std::string type;

	ros::NodeHandle nh;

	ros::Time lastUpdate;
	bool useHertz;
	float hertz;
};


#endif
