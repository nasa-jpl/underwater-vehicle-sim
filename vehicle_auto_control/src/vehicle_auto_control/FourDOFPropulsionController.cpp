#include <math.h>
#include <algorithm>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "vehicle_auto_control/PointPathAction.h"

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
    pointPathServer(controlNode, "point_path", false),
    plumeClient(controlNode.serviceClient<data_server::GetPlumeData>("data_server/get_plume"))
{
    velocityPub = vehicleNode.advertise<geometry_msgs::Twist>(propModuleName + "/command_velocity", 1000);

    if(dataModuleName != "")
    {
        hasVehicleData = true;
        dataSub = vehicleNode.subscribe(dataModuleName + "/data", 1, &FourDOFPropulsionController::getVehicleData, this);
    }

    velocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
    
    pointPathServer.registerGoalCallback(boost::bind(&FourDOFPropulsionController::goalPointPathCB, this));
    pointPathServer.registerPreemptCallback(boost::bind(&FourDOFPropulsionController::preemptPointPathCB, this));
    pointPathServer.start();
}

void FourDOFPropulsionController::getTargetVelocityCommand(const vehicle_auto_control::Velocity vel)
{
    targetHorzVelocity = fabs(vel.horizontalVelocity);
    targetRotVelocity = fabs(vel.rotationalVelocity);
    targetVertVelocity = fabs(vel.verticalVelocity);
}

void FourDOFPropulsionController::getVehicleData(const underwater_vehicle_sim::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
    latestVehicleDepth = data.h;
}

void FourDOFPropulsionController::update(void) 
{
    pointPathUpdate();
}

void FourDOFPropulsionController::goalPointPathCB(void)
{
    vehicle_auto_control::PointPathRosGoalConstPtr pointPathGoal = pointPathServer.acceptNewGoal();
    ROS_DEBUG("Point Path New Goal");
    currentPoint = 0;
    goingUp = true;
    pathPoints.clear();
    for(auto point: pointPathGoal->points)
    {
        pathPoints.emplace_back(point.x, point.y, point.z);
    }
    yoyo = pointPathGoal->yoyo;
    upperDepth = pointPathGoal->upperDepth;
    lowerDepth = pointPathGoal->lowerDepth;
}

void FourDOFPropulsionController::preemptPointPathCB(void)
{
    ROS_DEBUG("Point Path Action Preempted CB");

    //Stop the vehicle
    sendVelocityCommand(0, 0, 0, 0);

    currentPoint = 0;
    goingUp = true;
    pointPathServer.setPreempted();
}

void FourDOFPropulsionController::pointPathUpdate(void)
{
    vehicle_auto_control::PointPathRosFeedback feedback;
    vehicle_auto_control::PointPathRosResult result;

    if(!pointPathServer.isActive())
    {
        return;
    }

    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/world", "/" + vehicleName,
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("/world", "/" + vehicleName,  
                                 ros::Time(0), transform);
        if(currentPoint < pathPoints.size())
        {
            if(isAtPoint(transform, pathPoints[currentPoint], !yoyo))
            {
                currentPoint++;
            }
        }
        
        if(yoyo)
        {
            if(transform.getOrigin().getZ() + verticalError > upperDepth || transform.getOrigin().getZ() + verticalError >= 0)
            {
                goingUp = false;
            }
            else if(transform.getOrigin().getZ() - verticalError < lowerDepth || fabs(latestSonarDepth - minSeafloorDistance) <= verticalError)
            {
                goingUp = true;
            }
        }
        
        if(currentPoint < pathPoints.size())
        {
            double targetHeight = 0;
            if(yoyo)
            {
                targetHeight = goingUp ? upperDepth : lowerDepth;
            }
            else
            {
                targetHeight = pathPoints[currentPoint].getZ();
            }

            goToPoint(transform, pathPoints[currentPoint], targetHeight);
        }
        else
        {
            //Stop the vehicle
            sendVelocityCommand(0, 0, 0, 0);
        }
        
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


    feedback.currentPoint = currentPoint;
    feedback.goingUp = goingUp;
    pointPathServer.publishFeedback(feedback);

    if(currentPoint == pathPoints.size())
    {
        ROS_DEBUG("Point Path Action set succeeded");
        result.totalPoints = currentPoint;
        pointPathServer.setSucceeded(result);
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

bool FourDOFPropulsionController::isAtPoint(tf::Transform& location, tf::Vector3& point, bool useZ)
{
    double targetVertPosition = std::max(point.getZ(), latestVehicleDepth - latestSonarDepth + minSeafloorDistance);

    double xDifference = fabs(location.getOrigin().getX() - point.getX());
    double yDifference = fabs(location.getOrigin().getY() - point.getY());
    double zDifference = fabs(location.getOrigin().getZ() - targetVertPosition);

    return (!useZ || zDifference <= verticalError) && sqrt(yDifference * yDifference + xDifference * xDifference) <= lateralError;
}

void FourDOFPropulsionController::goToPoint(tf::StampedTransform& location, tf::Vector3& point, double targetHeight)
{
    geometry_msgs::PointStamped pointOut;
    transformPointToVehicleFrame(pointOut, location, point);
    tf::Vector3 vehicleForward(1, 0, 0);
    tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
    tf::Vector3 cross = vehicleForward.cross(targetPoint);

    double angle = vehicleForward.angle(targetPoint);       

    double newVertVel = scaleVerticalVelocity(location, targetHeight);
    double newRotVel = scaleRotationalVelocity(angle, cross.getZ());
    double newForwVel = scaleHorizontalVelocity(location, point);

    sendVelocityCommand(newForwVel, 0, newRotVel, newVertVel);
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
    double angleErrorScale = M_PI; //60 degrees 

    if(angleError >= angleErrorScale)
    {
        if(crossZ >= 0)
        {
            return targetRotVelocity;
        }
        else
        {
            return -targetRotVelocity;
        }
    }

    if(crossZ >= 0)
    {
        return targetRotVelocity * (angleError / angleErrorScale);
    }
    
    return -targetRotVelocity * (angleError / angleErrorScale);
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
}