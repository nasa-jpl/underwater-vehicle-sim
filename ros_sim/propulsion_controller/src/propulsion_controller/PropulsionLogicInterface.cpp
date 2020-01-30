#include "propulsion_controller/PropulsionLogicInterface.h"

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