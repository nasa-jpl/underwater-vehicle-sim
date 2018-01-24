#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "vehicles/PowerCapacityModule.h"

PowerCapacityModule::PowerCapacityModule(std::string name, ros::NodeHandle& parentNH) :
	GeneralModule(name, parentNH)
{

}

void PowerCapacityModule::update(std::string name, const ros::Time& lastTime, const tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	powerCapacity = powerCapacity - 0.1;
}
