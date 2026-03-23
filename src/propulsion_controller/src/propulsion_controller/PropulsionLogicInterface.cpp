#include "propulsion_controller/PropulsionLogicInterface.h"

PropulsionLogicInterface::PropulsionLogicInterface(VehicleInfo& vehicleInfo) :
	vehicleInfo(vehicleInfo),
    targetLinearVelocity(0,0,0),
    targetAngularVelocity(0,0,0)
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

void PropulsionLogicInterface::setVelocityXY(double xLinearVelocity, double yLinearVelocity, double zAngularVelocity) {
    targetLinearVelocity[0] = xLinearVelocity;
    targetLinearVelocity[1] = yLinearVelocity;
    targetAngularVelocity[2] = zAngularVelocity;
}

void PropulsionLogicInterface::setVelocityZ(double zLinearVelocity) {
    targetLinearVelocity[2] = zLinearVelocity;
}