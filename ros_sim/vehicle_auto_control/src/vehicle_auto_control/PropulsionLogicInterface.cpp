#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

PropulsionLogicInterface::PropulsionLogicInterface(VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo)
{}

std::unique_ptr<PropulsionLogicInterface> PropulsionLogicInterface::makePropulsionLogic(VehicleInfo& vehicleInfo)
{
	std::unique_ptr<PropulsionLogicInterface> logic;
	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		logic.reset(new FourDOFPropulsionLogic(vehicleInfo));
	}
	return logic;
}