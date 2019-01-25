#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

PropulsionLogicInterface::PropulsionLogicInterface(ros::NodeHandle vehicleNode, VehicleInfo& vehicleInfo) :
	vehicleNode(vehicleNode),
	vehicleInfo(vehicleInfo)
{}

std::unique_ptr<PropulsionLogicInterface> PropulsionLogicInterface::makePropulsionLogic(ros::NodeHandle vehicleNode, VehicleInfo& vehicleInfo)
{
	std::unique_ptr<PropulsionLogicInterface> logic;
	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		logic.reset(new FourDOFPropulsionLogic(vehicleNode, vehicleInfo));
	}
	return logic;
}