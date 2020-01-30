#include <math.h>
#include <algorithm>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "propulsion_controller/Velocity.h"
#include "uth/UthPropulsionLogic.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace underwater_autonomy;

UthPropulsionLogic::UthPropulsionLogic(VehicleInfo& vehicleInfo) :
    PropulsionLogicInterface(vehicleInfo),
    xyMovement(false),
	zMovement(false),
	avoidSeafloorCalls(0),
	goToXYCalls(0),
	goToZCalls(0),
	followHeadingCalls(0),
	stopXYCalls(0),
	stopZCalls(0),
	atXY(false),
	atZ(false)
{}

void UthPropulsionLogic::goToXY(VehiclePose& pose)
{
    xyMovement = true;
    goToXYCalls++;
}

void UthPropulsionLogic::followHeading(underwater_autonomy::VehiclePose& pose)
{
    xyMovement = true;
    followHeadingCalls++;
}

void UthPropulsionLogic::goToZ(VehiclePose& pose)
{
    zMovement = true;
    goToZCalls++;
}

void UthPropulsionLogic::avoidSeafloor(underwater_autonomy::VehiclePose& pose)
{
    zMovement = true;
    avoidSeafloorCalls++;
}

void UthPropulsionLogic::stopXY(void)
{
    xyMovement = false;
    stopXYCalls++;
}

void UthPropulsionLogic::stopZ(void)
{
    zMovement = false;
    stopZCalls++;
}

bool UthPropulsionLogic::isAtXY(VehiclePose& pose)
{
    return atXY;
}

bool UthPropulsionLogic::isAtZ(VehiclePose& pose)
{
    return atZ;
}

void UthPropulsionLogic::setTargetVelocity(const geometry_msgs::Twist vel)
{
    targetVelocity = vel;
}

void UthPropulsionLogic::processNewData(const underwater_vehicle_msgs::VehicleData data)
{
    newData = data;
}

bool UthPropulsionLogic::getXYMovement() {return xyMovement;}
bool UthPropulsionLogic::getZMovement() {return zMovement;}

int UthPropulsionLogic::getAvoidSeafloorCalls() {return avoidSeafloorCalls;}
int UthPropulsionLogic::getGoToXYCalls() {return goToXYCalls;}
int UthPropulsionLogic::getGoToZCalls() {return goToZCalls;}
int UthPropulsionLogic::getFollowHeadingCalls() {return followHeadingCalls;}
int UthPropulsionLogic::getStopXYCalls() {return stopXYCalls;}
int UthPropulsionLogic::getStopZCalls() {return stopZCalls;}

void UthPropulsionLogic::setAtXY(bool atXY) {this->atXY = atXY;}
void UthPropulsionLogic::setAtZ(bool atZ) {this->atZ = atZ;}

geometry_msgs::Twist UthPropulsionLogic::getTargetVelocity() {return targetVelocity;}

underwater_vehicle_msgs::VehicleData UthPropulsionLogic::getNewData() {return newData;}