#ifndef MODULE_H
#define MODULE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

class GeneralModule
{
public:
	GeneralModule(std::string name, ros::NodeHandle parentNH);

	virtual ~GeneralModule(){}

	virtual void update(std::string name, const ros::Time& lastTime, const tf::Vector3& position)=0;

	void updateAtRate(std::string name, const ros::Time& lastTime, const tf::Vector3& position);

	static std::unique_ptr<GeneralModule> makeGeneralModule(std::string moduleName, ros::NodeHandle& parentNH);

protected:
	ros::NodeHandle nh;

	ros::Time lastUpdate;
	bool useHertz;
	float hertz;
};


#endif
