#include <math.h>
#include <algorithm>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "data_server/GetPlumeData.h"

FourDOFPropulsionController::FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName) :
    PropulsionController(controlNode, vehicleNode, vehicleName),
    targetHorzVelocity(0),
    targetRotVelocity(0),
    targetVertVelocity(0),
    lateralError(5.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    latestVehicleDepth(0),
    minSeafloorDistance(10.0),
    lastForwardVelocity(0),
    lastLateralVelocity(0),
    lastRotVelocity(0),
    lastVertVelocity(0),
    goToXYServer(controlNode, "go_to_xy", false),
    goToZServer(controlNode, "go_to_z", false)
{
    velocityPub = vehicleNode.advertise<geometry_msgs::Twist>(propModuleName + "/command_velocity", 1000);

    if(dataModuleName != "")
    {
        hasVehicleData = true;
        dataSub = vehicleNode.subscribe(dataModuleName + "/data", 1, &FourDOFPropulsionController::getVehicleData, this);
    }

    velocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
    
    goToXYServer.registerGoalCallback(boost::bind(&FourDOFPropulsionController::goalGoToXYCB, this));
    goToXYServer.registerPreemptCallback(boost::bind(&FourDOFPropulsionController::preemptGoToXYCB, this));
    goToXYServer.start();

    goToZServer.registerGoalCallback(boost::bind(&FourDOFPropulsionController::goalGoToZCB, this));
    goToZServer.registerPreemptCallback(boost::bind(&FourDOFPropulsionController::preemptGoToZCB, this));
    goToZServer.start();

}

void FourDOFPropulsionController::getTargetVelocityCommand(const vehicle_auto_control::Velocity vel)
{
    targetHorzVelocity = fabs(vel.horizontalVelocity);
    targetRotVelocity = fabs(vel.rotationalVelocity);
    targetVertVelocity = fabs(vel.verticalVelocity);
}

void FourDOFPropulsionController::getVehicleData(const underwater_vehicle_msgs::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
    latestVehicleDepth = data.h;
}

void FourDOFPropulsionController::update(void) 
{ 
    if(goToXYServer.isActive())
    {
        goToXYUpdate();
    }

    if(goToZServer.isActive())
    {
        goToZUpdate();
    }
}

void FourDOFPropulsionController::cancelXYMovement(void)
{
     //Stop the vehicle in the xy direction
    sendVelocityCommand(0, 0, 0, lastVertVelocity);

    if(goToXYServer.isActive())
    {
        goToXYServer.setPreempted();
    }
}

void FourDOFPropulsionController::cancelZMovement(void)
{
    //Stop the vehicle in the z direction
    sendVelocityCommand(lastForwardVelocity, 
                        lastLateralVelocity, 
                        lastRotVelocity, 
                        0); //Vertical Velocity

    if(goToZServer.isActive())
    {
        goToZServer.setPreempted();
    }
}

void FourDOFPropulsionController::goalGoToXYCB(void)
{
    
    sendVelocityCommand(0, 0, 0, lastVertVelocity);
    vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
    
    targetX = goToXYGoal->x;
    targetY = goToXYGoal->y;
    ROS_INFO("GoToXY server accepted a new goal - x:%f y:%f", targetX, targetY);
}

void FourDOFPropulsionController::preemptGoToXYCB(void)
{
    ROS_DEBUG("GoToXY server current goal is preempted");
    cancelXYMovement();
}

void FourDOFPropulsionController::goToXYUpdate(void)
{
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/world", "/" + vehicleName,
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("/world", "/" + vehicleName,  
                                 ros::Time(0), transform);

        
        vehicle_auto_control::GoToXYRosFeedback feedback;
        feedback.x = transform.getOrigin().getX();
        feedback.y = transform.getOrigin().getY();
        goToXYServer.publishFeedback(feedback);

        if(isAtXY(transform))
        {
            double xDifference = fabs(transform.getOrigin().getX() - targetX);
            double yDifference = fabs(transform.getOrigin().getY() - targetY);

            ROS_INFO("Vehicle is at GoToXY goal location - Goal XY %f, %f; Vehicle XY %f, %f; Diff: %f",
                     transform.getOrigin().getX(),
                     transform.getOrigin().getY(),
                     targetX,
                     targetY,
                     sqrt(yDifference * yDifference + xDifference * xDifference));
            sendVelocityCommand(0.0, 
                                lastLateralVelocity, 
                                0.0, 
                                lastRotVelocity);

            vehicle_auto_control::GoToXYRosResult result;
            result.x = transform.getOrigin().getX();
            result.y = transform.getOrigin().getY();
            goToXYServer.setSucceeded(result);
        }
        else
        {
            goToXY(transform);

            //Z here to adjust to changing seafloor depth
            if(!goToZServer.isActive())
            {
                goToZ(transform);
            }
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

void FourDOFPropulsionController::goalGoToZCB(void)
{
    
    sendVelocityCommand(lastForwardVelocity, 
                        lastLateralVelocity, 
                        lastRotVelocity, 
                        0); //Vertical Velocity

    vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();

    targetZ = goToZGoal->z;
    ROS_INFO("GoToZ server accepted a new goal - z: %f", targetZ);
}

void FourDOFPropulsionController::preemptGoToZCB(void)
{
    ROS_INFO("GoToZ server current goal is preempted");
    cancelZMovement();
}

void FourDOFPropulsionController::goToZUpdate(void)
{
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/world", "/" + vehicleName,
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("/world", "/" + vehicleName,  
                                 ros::Time(0), transform);

        vehicle_auto_control::GoToZRosFeedback feedback;
        feedback.z = transform.getOrigin().getZ();
        goToZServer.publishFeedback(feedback);

        if(isAtZ(transform))
        {
            double targetVertPosition = std::max(targetZ, latestVehicleDepth - latestSonarDepth + minSeafloorDistance);

            double zDifference = fabs(transform.getOrigin().getZ() - targetVertPosition);

            ROS_INFO("Vehicle is at GoToZ goal location - Target Z: %f; Vehicle Z: %f; Transform Time: %f; Current Time: %f; Difference: %f",
                     targetVertPosition,
                     transform.getOrigin().getZ(),
                     transform.stamp_.toSec(),
                     ros::Time::now().toSec(),
                     zDifference);
            sendVelocityCommand(lastForwardVelocity, 
                                lastLateralVelocity, 
                                lastRotVelocity, 
                                0.0);

            vehicle_auto_control::GoToZRosResult result;
            result.z = transform.getOrigin().getZ();
            goToZServer.setSucceeded(result); 
        }
        else
        {
            goToZ(transform);
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

void FourDOFPropulsionController::transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform, tf::Vector3& point)
{
    geometry_msgs::PointStamped pointIn;
    
    pointIn.header.stamp = transform.stamp_;
    pointIn.header.frame_id = "/world";
    pointIn.point.x = point.getX();
    pointIn.point.y = point.getY();
    pointIn.point.z = point.getZ();

    listener.transformPoint("/" + vehicleName, pointIn, pointOut);
}

bool FourDOFPropulsionController::isAtXY(tf::Transform& location)
{
    double xDifference = fabs(location.getOrigin().getX() - targetX);
    double yDifference = fabs(location.getOrigin().getY() - targetY);

    return sqrt(yDifference * yDifference + xDifference * xDifference) <= lateralError;
}

bool FourDOFPropulsionController::isAtZ(tf::Transform& location)
{
    double targetVertPosition = std::max(targetZ, latestVehicleDepth - latestSonarDepth + minSeafloorDistance);

    double zDifference = fabs(location.getOrigin().getZ() - targetVertPosition);

    return zDifference <= verticalError;
}


void FourDOFPropulsionController::goToXY(tf::StampedTransform& location)
{
    geometry_msgs::PointStamped pointOut;
    tf::Vector3 point(targetX, targetY, 0);

    transformPointToVehicleFrame(pointOut, location, point);
    tf::Vector3 vehicleForward(1, 0, 0);
    tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
    tf::Vector3 cross = vehicleForward.cross(targetPoint);

    double angle = vehicleForward.angle(targetPoint);       
    double newRotVel = lastRotVelocity;
    if(std::isfinite(angle))
    {
        newRotVel = scaleRotationalVelocity(angle, cross.getZ());
        lastRotVelocity = newRotVel;
    }

    double newForwVel = scaleHorizontalVelocity(location, point);
    lastForwardVelocity = newForwVel;
    sendVelocityCommand(newForwVel, 
                        lastLateralVelocity, 
                        newRotVel, 
                        lastRotVelocity);
}

void FourDOFPropulsionController::goToZ(tf::StampedTransform& location)
{
    double newVertVel = scaleVerticalVelocity(location, targetZ);

    lastVertVelocity = newVertVel;
    sendVelocityCommand(lastForwardVelocity, 
                        lastLateralVelocity, 
                        lastRotVelocity, 
                        newVertVel);
}

double FourDOFPropulsionController::scaleHorizontalVelocity(tf::Transform& location, tf::Vector3& point)
{
    double xDifference = fabs(location.getOrigin().getX() - point.getX());
    double yDifference = fabs(location.getOrigin().getY() - point.getY());
    double xyError = sqrt(yDifference * yDifference + xDifference * xDifference);

    double horizontalScaleError = 100;

    if(xyError >= horizontalScaleError)
    {
        return targetHorzVelocity;
    }
    
    return targetHorzVelocity * (xyError / horizontalScaleError);
}

double FourDOFPropulsionController::scaleVerticalVelocity(tf::Transform& location, double targetHeight)
{
    double targetVertPosition = std::max(targetHeight, latestVehicleDepth - latestSonarDepth + minSeafloorDistance);
    double zDifference = fabs(location.getOrigin().getZ() - targetVertPosition);
    double verticalErrorScale = 15;

    int sign = 0;
    if(targetVertPosition >= location.getOrigin().getZ())
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    if(zDifference >= verticalErrorScale)
    {
        return targetVertVelocity * sign;
    }
    
    return targetVertVelocity * (zDifference / verticalErrorScale) * sign;
}

double FourDOFPropulsionController::scaleRotationalVelocity(double angleError, double crossZ)
{
  //  ROS_INFO("SCALE ROTATE: %f %f %f", targetRotVelocity, angleError, crossZ);
    double angleErrorScale = M_PI; //60 degrees 

    if(angleError >= angleErrorScale)
    {
        if(crossZ >= 0)
        {
            return -targetRotVelocity;
        }
        else
        {
            return targetRotVelocity;
        }
    }

    if(crossZ >= 0)
    {
        return -targetRotVelocity * (angleError / angleErrorScale);
    }

    return targetRotVelocity * (angleError / angleErrorScale);
}

void FourDOFPropulsionController::sendVelocityCommand(double cmdForwardVelocity, double cmdLateralVelocity, double cmdRotVelocity, double cmdVertVelocity)
{
    geometry_msgs::Twist commandMsg;
    geometry_msgs::Vector3 lin;
    geometry_msgs::Vector3 rot;

    lin.x = cmdForwardVelocity;
    lin.y = cmdLateralVelocity;
    lin.z = cmdVertVelocity;

    rot.x = 0;
    rot.y = 0;
    rot.z = cmdRotVelocity;

    commandMsg.linear = lin;
    commandMsg.angular = rot;
    velocityPub.publish(commandMsg);
  /*  ROS_INFO("Send velocity command to vehicle - %f %f %f %f",
        cmdForwardVelocity,
        cmdLateralVelocity,
        cmdRotVelocity,
        cmdVertVelocity);*/
}