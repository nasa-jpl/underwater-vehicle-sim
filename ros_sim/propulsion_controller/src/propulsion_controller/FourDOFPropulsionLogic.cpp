#include <math.h>
#include <algorithm>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "propulsion_controller/Velocity.h"
#include "propulsion_controller/FourDOFPropulsionLogic.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace underwater_autonomy;

FourDOFPropulsionLogic::FourDOFPropulsionLogic(VehicleInfo& vehicleInfo,
                                               underwater_autonomy::LinearPiecewise forwardThruster,
                                               underwater_autonomy::LinearPiecewise verticalThruster,
                                               underwater_autonomy::LinearPiecewise rudder) :
    PropulsionLogicInterface(vehicleInfo),
    forwardThruster(forwardThruster),
    verticalThruster(verticalThruster),
    rudder(rudder),
    lateralError(25.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    minSeafloorDistance(3.0),
    horizontalErrorScale(25),
    verticalErrorScale(5),
    angleErrorScale(3.14159265359) //180 degrees
{
    ros::NodeHandle nh;

    forwardThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_forward_thruster", 1000);
    lateralThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_lateral_thruster", 1000);
    verticalThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_vertical_thruster", 1000);
    rudderPub = vehicleNode.advertise<std_msgs::Float64>("command_rudder", 1000);
}

void FourDOFPropulsionLogic::goToXY(VehiclePose& pose)
{      
    Eigen::Vector3d point(targetX, targetY, 0);

    //transform point to vehicle frame
    point = point - pose.getPosition();
    point = pose.getOrientation().inverse() * point;
    point[2] = 0; //Zero z as we do not care about it

    double angle = atan2(point[1], point[0]);

    double targetForwardVelocity = scaleHorizontalVelocity(point.norm());
    double currentForwardVelocity = pose.getLinearVelocity()[0];
    if(std::isfinite(currentForwardVelocity) &&
       std::isfinite(targetForwardVelocity))
    {
        std_msgs::Float64 data;
        data.data = forwardThruster.getY(targetForwardVelocity).y;
        forwardThrustPub.publish(data);
    }

    if(std::isfinite(angle))
    {
        std_msgs::Float64 data;
        data.data = rudder.getY(scaleRotationalVelocity(angle)).y;
        rudderPub.publish(data);
    }
}

void FourDOFPropulsionLogic::followHeading(underwater_autonomy::VehiclePose& pose)
{
    double currentForwardVelocity = pose.getLinearVelocity()[0];
    double currentAngle = pose.getOrientation().toRotationMatrix().eulerAngles(0, 1, 2)[2];

    //Get and normalize the angle difference
    double angle = targetHeading - currentAngle;
    angle = std::fmod(angle, 2 * M_PI);
    angle = std::fmod(angle + (2 * M_PI), 2 * M_PI);
    if(angle > M_PI)
    {
        angle -= 2 * M_PI;
    }

    if(std::isfinite(currentForwardVelocity) &&
       std::isfinite(targetLinearVelocity.x()))
    {
        std_msgs::Float64 data;
        data.data = forwardThruster.getY(targetLinearVelocity.x()).y;
        forwardThrustPub.publish(data);
    }
    if(std::isfinite(angle))
    {
        std_msgs::Float64 data;
        data.data = rudder.getY(scaleRotationalVelocity(angle)).y;
        rudderPub.publish(data);
    }
}

void FourDOFPropulsionLogic::goToZ(VehiclePose& pose)
{
    double targetVertPosition = std::min(targetZ, (pose.getPosition()[2] + latestSonarDepth) - minSeafloorDistance);
    double targetVertVelocity = scaleVerticalVelocity(targetVertPosition - pose.getPosition()[2]);

    double currentVertVelocity = pose.getLinearVelocity()[2];

    if(std::isfinite(currentVertVelocity) &&
       std::isfinite(targetVertVelocity))
    {
        std_msgs::Float64 data;
        data.data = verticalThruster.getY(targetVertVelocity).y;
        verticalThrustPub.publish(data);
    }
}

void FourDOFPropulsionLogic::avoidSeafloor(underwater_autonomy::VehiclePose& pose)
{
    if(minSeafloorDistance > latestSonarDepth)
    {
        double targetMaxDepth = (pose.getPosition()[2] + latestSonarDepth) - minSeafloorDistance;
        double targetVertVelocity = scaleVerticalVelocity(targetMaxDepth - pose.getPosition()[2]);
        double currentVertVelocity = pose.getLinearVelocity()[2];
        if(std::isfinite(currentVertVelocity) &&
           std::isfinite(targetVertVelocity))
        {
            std_msgs::Float64 data;
            data.data = verticalThruster.getY(targetVertVelocity).y;
            verticalThrustPub.publish(data);
        }
    }
    else
    {
        std_msgs::Float64 data;
        data.data = 0;
        verticalThrustPub.publish(data);
    }    
}

void FourDOFPropulsionLogic::stopXY(void)
{
    std_msgs::Float64 data;
    data.data = 0;
    forwardThrustPub.publish(data);
    lateralThrustPub.publish(data);
    rudderPub.publish(data);
}

void FourDOFPropulsionLogic::stopZ(void)
{
    std_msgs::Float64 data;
    data.data = 0;
    verticalThrustPub.publish(data);
}

bool FourDOFPropulsionLogic::isAtXY(underwater_autonomy::VehiclePose& pose)
{
    tf2::Vector3 point(targetX - pose.getPosition()[0], targetY - pose.getPosition()[1], 0);
    return abs(point.length()) <= lateralError;
}

bool FourDOFPropulsionLogic::isAtZ(underwater_autonomy::VehiclePose& pose)
{
    double targetVertPosition = std::min(targetZ, (pose.getPosition()[2] + latestSonarDepth) - minSeafloorDistance);
    tf2::Vector3 point(0, 0, targetVertPosition - pose.getPosition()[2]);

    //We only care about z
    return abs(point.z()) <= verticalError;
}

void FourDOFPropulsionLogic::processNewData(const underwater_vehicle_msgs::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
}

double FourDOFPropulsionLogic::scaleHorizontalVelocity(double distance)
{
    if(distance >= horizontalErrorScale)
    {
        return targetLinearVelocity.x();
    }
    
    return targetLinearVelocity.x() * (distance / horizontalErrorScale);
}

double FourDOFPropulsionLogic::scaleRotationalVelocity(double angle)
{
    int sign = 0;
    if(angle >= 0)
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    if(abs(angle) >= angleErrorScale)
    {
        return targetAngularVelocity.z() * sign;
    }

    return targetAngularVelocity.z() * (angle / angleErrorScale);
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