#include <math.h>
#include <algorithm>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

FourDOFPropulsionLogic::FourDOFPropulsionLogic() :
    lateralError(5.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    latestVehicleDepth(0),
    minSeafloorDistance(10.0),
    angleErrorScale(M_PI / 2),
    horizontalScaleError(100),
    verticalErrorScale(15)
{}

const geometry_msgs::Twist FourDOFPropulsionLogic::goToXYTwist(tf2::Stamped<tf2::Transform>& NEDToVehicle)
{
    tf2::Vector3 point(targetX, targetY, 0);
    point = NEDToVehicle * point;
    point.setZ(0); //Ignore z as we are only interested in x and y

    double angle = atan2(point.getY(), point.getX());
    
    if(std::isfinite(angle))
    {
        lastAngularVelocity.z = scaleRotationalVelocity(angle);
    }

    lastLinearVelocity.x = scaleHorizontalVelocity(point.length());

    geometry_msgs::Twist newTwist;
    newTwist.linear = lastLinearVelocity;
    newTwist.angular = lastAngularVelocity;
    return newTwist;
}

const geometry_msgs::Twist FourDOFPropulsionLogic::goToZTwist(tf2::Stamped<tf2::Transform>& NEDToVehicle)
{
    double targetVertPosition = std::min(targetZ, (latestVehicleDepth + latestSonarDepth) - minSeafloorDistance);
    tf2::Vector3 point(0, 0, targetVertPosition);
    point = NEDToVehicle * point;

    lastLinearVelocity.z = scaleVerticalVelocity(point.getZ());

    geometry_msgs::Twist newTwist;
    newTwist.linear = lastLinearVelocity;
    newTwist.angular = lastAngularVelocity;
    return newTwist;
}

const geometry_msgs::Twist FourDOFPropulsionLogic::stopXYTwist(void)
{
	lastLinearVelocity.x = 0;
    lastLinearVelocity.y = 0;
    lastAngularVelocity.z = 0;

    geometry_msgs::Twist newTwist;
    newTwist.linear = lastLinearVelocity;
    newTwist.angular = lastAngularVelocity;

    return newTwist;
}

const geometry_msgs::Twist FourDOFPropulsionLogic::stopZTwist(void)
{
    lastLinearVelocity.z = 0;

    geometry_msgs::Twist newTwist;
    newTwist.linear = lastLinearVelocity;
    newTwist.angular = lastAngularVelocity;

    return newTwist;
}

void FourDOFPropulsionLogic::setTargetXY(double x, double y)
{
    targetX = x;
    targetY = y;
}

void FourDOFPropulsionLogic::setTargetZ(double z)
{
    targetZ = z;
}

bool FourDOFPropulsionLogic::isAtXY(tf2::Stamped<tf2::Transform>& NEDToVehicle)
{
    tf2::Vector3 point(targetX, targetY, 0);
    point = NEDToVehicle * point;
    point.setZ(0); //Zero out z as we are only interested in xy

    return abs(point.length()) <= lateralError;
}

bool FourDOFPropulsionLogic::isAtZ(tf2::Stamped<tf2::Transform>& NEDToVehicle)
{
    double targetVertPosition = std::min(targetZ, (latestVehicleDepth + latestSonarDepth) - minSeafloorDistance);
    tf2::Vector3 point(0, 0, targetVertPosition);
    point = NEDToVehicle * point;

    //We only care about z
    return abs(point.z()) <= verticalError;
}

void FourDOFPropulsionLogic::setTargetVelocity(const geometry_msgs::Twist vel)
{
    tf2::fromMsg(vel.linear, targetLinearVelocity);
    tf2::fromMsg(vel.angular, targetAngularVelocity);
}

void FourDOFPropulsionLogic::processNewData(const underwater_vehicle_msgs::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
    latestVehicleDepth = data.h;
}

double FourDOFPropulsionLogic::scaleHorizontalVelocity(double distance)
{
    if(distance >= horizontalScaleError)
    {
        return targetLinearVelocity.x();
    }
    
    return targetLinearVelocity.x() * (distance / horizontalScaleError);
}

double FourDOFPropulsionLogic::scaleVerticalVelocity(double zDifference)
{
    int sign = 0;
    if(zDifference >= 0)
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    if(abs(zDifference) >= verticalErrorScale)
    {
        return targetLinearVelocity.z() * sign;
    }
    
    return targetLinearVelocity.z() * (zDifference / verticalErrorScale);
}

double FourDOFPropulsionLogic::scaleRotationalVelocity(double angle)
{
    if(abs(angle) >= angleErrorScale)
    {
        if(angle >= 0)
        {
            return targetAngularVelocity.z();
        }
        else
        {
            return -targetAngularVelocity.z();
        }
    }
    return targetAngularVelocity.z() * (angle / angleErrorScale);
}