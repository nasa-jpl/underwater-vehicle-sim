#include <math.h>
#include <algorithm>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

#include "std_msgs/Float64.h"

FourDOFPropulsionLogic::FourDOFPropulsionLogic(ros::NodeHandle vehicleNode, VehicleInfo& vehicleInfo) :
    PropulsionLogicInterface(vehicleNode, vehicleInfo),
    lateralError(5.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    latestVehicleDepth(0),
    minSeafloorDistance(10.0),
    angleErrorScale(M_PI),
    horizontalScaleError(100),
    verticalErrorScale(15),
    targetLinearVelocity(0,0,0),
    targetAngularVelocity(0,0,0),
    lastForwardThrust(0),
	lastRudder(0),
	lastVertThrust(0)
{
    forwardThrustPub = vehicleNode.advertise<std_msgs::Float64>(vehicleInfo.getPropModuleName() + "/command_forward_thruster", 1000);
    lateralThrustPub = vehicleNode.advertise<std_msgs::Float64>(vehicleInfo.getPropModuleName() + "/command_lateral_thruster", 1000);
    verticalThrustPub = vehicleNode.advertise<std_msgs::Float64>(vehicleInfo.getPropModuleName() + "/command_vertical_thruster", 1000);
    rudderPub = vehicleNode.advertise<std_msgs::Float64>(vehicleInfo.getPropModuleName() + "/command_rudder", 1000);
}

const void FourDOFPropulsionLogic::goToXY(VehiclePose& pose)
{
    Eigen::Vector3d point(targetX, targetY, 0);

    //transform point to vehicle frame
    point = pose.getPosition() - point;
    point = pose.getOrientation().inverse() * point;
    point[2] = 0; //Zero of z as we do not care about it

    double angle = atan2(point[1], point[0]);
    
    double targetAngularVelocity = std::isfinite(angle) ? scaleRotationalVelocity(angle) : 0;
    double targetForwardVelocity = scaleHorizontalVelocity(point.norm());

    double currentAngularVelocity = pose.getAngularVelocity()[2];
    double currentForwardVelocity = (pose.getOrientation() * pose.getLinearVelocity())[0];

    //This is a temporary measure to test command before changing things to incorporate a PID controller
    std_msgs::Float64 forwardThrust;
    forwardThrust.data = targetForwardVelocity;
    forwardThrustPub.publish(forwardThrust);

    std_msgs::Float64 rudder;
    rudder.data = targetAngularVelocity;
    rudderPub.publish(rudder);
}

const void FourDOFPropulsionLogic::goToZ(VehiclePose& pose)
{
    double targetVertPosition = std::min(targetZ, (latestVehicleDepth + latestSonarDepth) - minSeafloorDistance);
    double targetVertVelocity = scaleVerticalVelocity(targetVertPosition - pose.getPosition()[2]);

    double currentVertVelocity = pose.getLinearVelocity()[2];

    //This is a temporary measure to test command before changing things to incorporate a PID controller
    std_msgs::Float64 vertThrust;
    vertThrust.data = targetVertVelocity;
    verticalThrustPub.publish(vertThrust);
}

const void FourDOFPropulsionLogic::stopXY(void)
{
    std_msgs::Float64 msg;
    msg.data = 0;
    forwardThrustPub.publish(msg);
    lateralThrustPub.publish(msg);
    rudderPub.publish(msg);
}

const void FourDOFPropulsionLogic::stopZ(void)
{
    std_msgs::Float64 msg;
    msg.data = 0;
    verticalThrustPub.publish(msg);
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

bool FourDOFPropulsionLogic::isAtXY(VehiclePose& pose)
{
    tf2::Vector3 point(targetX - pose.getPosition()[0], targetY - pose.getPosition()[1], 0);
    return abs(point.length()) <= lateralError;
}

bool FourDOFPropulsionLogic::isAtZ(VehiclePose& pose)
{
    double targetVertPosition = std::min(targetZ, (latestVehicleDepth + latestSonarDepth) - minSeafloorDistance);
    tf2::Vector3 point(0, 0, targetVertPosition - pose.getPosition()[2]);

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