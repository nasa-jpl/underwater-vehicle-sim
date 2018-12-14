#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

std::unique_ptr<PropulsionLogicInterface> PropulsionLogicInterface::makePropulsionLogic(VehicleInfo info)
{
	std::unique_ptr<PropulsionLogicInterface> logic;
	if(info.getPropModuleType() == "FourDOFPropulsion")
	{
		logic.reset(new FourDOFPropulsionLogic());
	}
	return logic;
}
