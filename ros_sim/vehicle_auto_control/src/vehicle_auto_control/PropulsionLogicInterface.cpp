#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

PropulsionLogicInterface::PropulsionLogicInterface(VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo)
{}

void PropulsionLogicInterface::setTargetXY(double x, double y)
{
    targetX = x;
    targetY = y;
}

void PropulsionLogicInterface::setTargetZ(double z)
{
    targetZ = z;
}

double PropulsionLogicInterface::getTargetX()
{
    return targetX;
}

double PropulsionLogicInterface::getTargetY()
{
    return targetY;
}

double PropulsionLogicInterface::getTargetZ()
{
    return targetZ;
}

double PropulsionLogicInterface::getFollowHeading()
{
	return targetHeading;
}

void PropulsionLogicInterface::setFollowHeading(double heading)
{
    targetHeading = heading;
}

std::unique_ptr<PropulsionLogicInterface> PropulsionLogicInterface::makePropulsionLogic(VehicleInfo& vehicleInfo)
{
	std::unique_ptr<PropulsionLogicInterface> logic;
	if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
	{
		logic.reset(new FourDOFPropulsionLogic(vehicleInfo));
	}
	return logic;
}