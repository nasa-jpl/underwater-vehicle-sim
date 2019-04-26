#include <math.h>
#include <algorithm>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace underwater_autonomy;

FourDOFPropulsionLogic::FourDOFPropulsionLogic(VehicleInfo& vehicleInfo) :
    PropulsionLogicInterface(vehicleInfo),
    lateralError(5.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    latestVehicleDepth(0),
    minSeafloorDistance(3.0),
    angleErrorScale(M_PI),
    horizontalScaleError(25),
    verticalErrorScale(15),
    targetLinearVelocity(0,0,0),
    targetAngularVelocity(0,0,0),
    lastForwardThrust(0),
	lastRudder(0),
	lastVertThrust(0),
    xyEnabled(false),
    zEnabled(false)
{
    ros::NodeHandle nh;

    forwardThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_forward_thruster", 1000);
    lateralThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_lateral_thruster", 1000);
    verticalThrustPub = vehicleNode.advertise<std_msgs::Float64>("command_vertical_thruster", 1000);
    rudderPub = vehicleNode.advertise<std_msgs::Float64>("command_rudder", 1000);

    forwardThrusterState = nh.advertise<std_msgs::Float64>("forward_thruster/state", 10);
	forwardThrusterSetpoint = nh.advertise<std_msgs::Float64>("forward_thruster/setpoint", 10);
	forwardThrusterEnable = nh.advertise<std_msgs::Bool>("forward_thruster/pid_enable", 10);
	forwardThrusterControlEffort = nh.subscribe("forward_thruster/control_effort", 
                                                1, 
                                                &FourDOFPropulsionLogic::forwardThrusterControlEffortCB, 
                                                this);

    lateralThrusterState = nh.advertise<std_msgs::Float64>("lateral_thruster/state", 10);
	lateralThrusterSetpoint = nh.advertise<std_msgs::Float64>("lateral_thruster/setpoint", 10);
	lateralThrusterEnable = nh.advertise<std_msgs::Bool>("lateral_thruster/pid_enable", 10);
	lateralThrusterControlEffort = nh.subscribe("lateral_thruster/control_effort", 
                                                1,
                                                &FourDOFPropulsionLogic::lateralThrusterControlEffortCB, 
                                                this);

    verticalThrusterState = nh.advertise<std_msgs::Float64>("vertical_thruster/state", 10);
	verticalThrusterSetpoint = nh.advertise<std_msgs::Float64>("vertical_thruster/setpoint", 10);
	verticalThrusterEnable = nh.advertise<std_msgs::Bool>("vertical_thruster/pid_enable", 10);
	verticalThrusterControlEffort = nh.subscribe("vertical_thruster/control_effort", 
                                                 1, 
                                                 &FourDOFPropulsionLogic::verticalThrusterControlEffortCB, 
                                                 this);

    rudderState = nh.advertise<std_msgs::Float64>("rudder/state", 10);
	rudderSetpoint = nh.advertise<std_msgs::Float64>("rudder/setpoint", 10);
	rudderEnable = nh.advertise<std_msgs::Bool>("rudder/pid_enable", 10);
	rudderControlEffort = nh.subscribe("rudder/control_effort", 
                                       1, 
                                       &FourDOFPropulsionLogic::rudderControlEffortCB, 
                                       this);
}

const void FourDOFPropulsionLogic::goToXY(VehiclePose& pose)
{
    if(!xyEnabled)
    {
        std_msgs::Bool enableMsg;
        enableMsg.data = true;
        forwardThrusterEnable.publish(enableMsg);
        rudderEnable.publish(enableMsg);
        xyEnabled = true;
    }
    
    
    Eigen::Vector3d point(targetX, targetY, 0);

    //transform point to vehicle frame
    point = point - pose.getPosition();
    point = pose.getOrientation().inverse() * point;
    point[2] = 0; //Zero z as we do not care about it

    double angle = atan2(point[1], point[0]);

    double targetForwardVelocity = scaleHorizontalVelocity(point.norm());

    //rotate angular velocity from body frame into world frame and get angular velocity corresponding to heading
    double currentAngularVelocity =  (pose.getOrientation() * pose.getAngularVelocity())[2];
    double currentForwardVelocity = pose.getLinearVelocity()[0];

    if(std::isfinite(currentForwardVelocity))
    {
        std_msgs::Float64 forwardStateMsg;
        forwardStateMsg.data = currentForwardVelocity;
        forwardThrusterState.publish(forwardStateMsg);

        std_msgs::Float64 forwardSetpointMsg;
        forwardSetpointMsg.data = targetForwardVelocity;
        forwardThrusterSetpoint.publish(forwardSetpointMsg);
    }
    
    if(std::isfinite(angle))
    {
        std_msgs::Float64 angularStateMsg;
        angularStateMsg.data = angle;
        rudderState.publish(angularStateMsg);

        std_msgs::Float64 angularSetpointMsg;
        angularSetpointMsg.data = 0;
        rudderSetpoint.publish(angularSetpointMsg);
    }
    
}

const void FourDOFPropulsionLogic::goToZ(VehiclePose& pose)
{
    if(!zEnabled)
    {
        std_msgs::Bool enableMsg;
        enableMsg.data = true;
        verticalThrusterEnable.publish(enableMsg);
        zEnabled = true;
    }

    double targetVertPosition = std::min(targetZ, (latestVehicleDepth + latestSonarDepth) - minSeafloorDistance);
    double targetVertVelocity = scaleVerticalVelocity(targetVertPosition - pose.getPosition()[2]);

    double currentVertVelocity = pose.getLinearVelocity()[2];

    if(std::isfinite(currentVertVelocity))
    {
        std_msgs::Float64 verticalStateMsg;
        verticalStateMsg.data = currentVertVelocity;
        verticalThrusterState.publish(verticalStateMsg);

        std_msgs::Float64 verticalSetpointMsg;
        verticalSetpointMsg.data = targetVertVelocity;
        verticalThrusterSetpoint.publish(verticalSetpointMsg);
    }
}

const void FourDOFPropulsionLogic::stopXY(void)
{
    xyEnabled = false;
    std_msgs::Bool enableMsg;
    enableMsg.data = false;
    forwardThrusterEnable.publish(enableMsg);
    rudderEnable.publish(enableMsg);

    std_msgs::Float64 msg;
    msg.data = 0;
    forwardThrustPub.publish(msg);
    lateralThrustPub.publish(msg);
    rudderPub.publish(msg);
}

const void FourDOFPropulsionLogic::stopZ(void)
{
    zEnabled = false;
    std_msgs::Bool enableMsg;
    enableMsg.data = false;
    verticalThrusterEnable.publish(enableMsg);

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

void FourDOFPropulsionLogic::forwardThrusterControlEffortCB(std_msgs::Float64 data)
{
    if(xyEnabled)
    {
        forwardThrustPub.publish(data);
    }
}

void FourDOFPropulsionLogic::lateralThrusterControlEffortCB(std_msgs::Float64 data)
{
    if(xyEnabled)
    {
        lateralThrustPub.publish(data);
    }
}

void FourDOFPropulsionLogic::verticalThrusterControlEffortCB(std_msgs::Float64 data)
{
    if(zEnabled)
    {
        verticalThrustPub.publish(data);
    }
}

void FourDOFPropulsionLogic::rudderControlEffortCB(std_msgs::Float64 data)
{
    if(xyEnabled)
    {
        rudderPub.publish(data);
    }
}