#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "vehicle_auto_control/PointPath.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "geometry_msgs/PointStamped.h"

FourDOFPropulsionController::FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName) :
	PropulsionController(controlNode, vehicleNode, vehicleName),
	targetHorzVelocity(0),
	targetRotVelocity(0),
	targetVertVelocity(0),
	currentPoint(0),
	lateralError(1.0),
	verticalError(0.25),
	rotationalError(0.0174533),
	goingUp(true)
{
	commandPointPathSub = controlNode.subscribe("command_point_path", 1, &FourDOFPropulsionController::getPointPathCommand, this);
	commandYoYoPointPathSub = controlNode.subscribe("command_yoyo_point_path", 1, &FourDOFPropulsionController::getYoYoPointPathCommand, this);
	commandTargetVelocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
	velocityPub = vehicleNode.advertise<geometry_msgs::Twist>("command_velocity", 1000);
}

void FourDOFPropulsionController::getPointPathCommand(const vehicle_auto_control::PointPath vel)
{
	currentTask = "PointPath";
	pointPath.clear();

	for(geometry_msgs::Point point : vel.points)
	{
		pointPath.emplace_back(point.x, point.y, point.z);
	}
	currentPoint = 0;
}

void FourDOFPropulsionController::getTargetVelocityCommand(const vehicle_auto_control::Velocity vel)
{
	targetHorzVelocity = vel.horizontalVelocity;
	targetRotVelocity = vel.rotationalVelocity;
	targetVertVelocity = vel.verticalVelocity;
}

void FourDOFPropulsionController::getYoYoPointPathCommand(const vehicle_auto_control::YoYoPointPath vel)
{
	currentTask = "YoYoPointPath";
	pointPath.clear();

	for(geometry_msgs::Point point : vel.points)
	{
		pointPath.emplace_back(point.x, point.y, point.z);
	}
	currentPoint = 0;

	yoyoUpperDepth = vel.upperDepth;
	yoyoLowerDepth = vel.lowerDepth;
}

void FourDOFPropulsionController::update()
{
	if(currentTask == "PointPath")
	{
		pointPathController();
	}
	else if(currentTask == "YoYoPointPath")
	{
		yoyoPointPathController();
	}
}

void FourDOFPropulsionController::pointPathController()
{
	double newVertVel = 0;
	double newRotVel = 0;
	double newForwVel = 0;

	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("/world", "/" + vehicleName,  
								 ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}

	if(currentPoint < pointPath.size())
	{
		if(isAtPoint(transform, pointPath[currentPoint]))
		{
			currentPoint++;
		}
	}

	if(currentPoint < pointPath.size())
	{
		//Set vertical
		if(pointPath[currentPoint].getZ() - transform.getOrigin().getZ()  > verticalError)
		{
			newVertVel = targetVertVelocity;
		}
		else if(pointPath[currentPoint].getZ() - transform.getOrigin().getZ() < -verticalError)
		{
			newVertVel = -targetVertVelocity;
		}

		geometry_msgs::PointStamped pointOut;
		transformPointToVehicleFrame(pointOut, transform);

		tf::Vector3 vehicleForward(1, 0, 0);
		tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
		tf::Vector3 cross = vehicleForward.cross(targetPoint);

		double angle = vehicleForward.angle(targetPoint);		

		if(angle >= rotationalError)
		{
			if(cross.getZ() > 0)
			{
				newRotVel = targetRotVelocity;
			}
			else if(cross.getZ() < 0)
			{
				newRotVel = -targetRotVelocity;
			}
			
		}

		//Set forward velocity
		newForwVel = targetHorzVelocity;
	}
	
	//Create velocity command message and send it to the propulsion module
	sendVelocityCommand(newForwVel, 0, newRotVel, newVertVel);

}

void FourDOFPropulsionController::yoyoPointPathController()
{
	double newVertVel = 0;
	double newRotVel = 0;
	double newForwVel = 0;

	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("/world", "/" + vehicleName,  
								 ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}

	if(currentPoint < pointPath.size())
	{
		if(isAtPoint(transform, pointPath[currentPoint]))
		{
			currentPoint++;
		}
	}

	//Check yoyo direction
	if(transform.getOrigin().getZ() + verticalError > yoyoUpperDepth)
	{
		goingUp = false;
	}
	else if(transform.getOrigin().getZ() - verticalError < yoyoLowerDepth)
	{
		goingUp = true;
	}

	if(currentPoint < pointPath.size())
	{
		//Set vertical
		if(goingUp)
		{
			newVertVel = targetVertVelocity;	
		}
		else
		{
			newVertVel = -targetVertVelocity;
		}

		
		geometry_msgs::PointStamped pointOut;
		transformPointToVehicleFrame(pointOut, transform);

		tf::Vector3 vehicleForward(1, 0, 0);
		tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
		tf::Vector3 cross = vehicleForward.cross(targetPoint);

		double angle = vehicleForward.angle(targetPoint);		

		if(angle >= rotationalError)
		{
			if(cross.getZ() > 0)
			{
				newRotVel = targetRotVelocity;
			}
			else if(cross.getZ() < 0)
			{
				newRotVel = -targetRotVelocity;
			}
			
		}

		//Set forward velocity
		newForwVel = targetHorzVelocity;
	}
	
	//Create velocity command message and send it to the propulsion module
	sendVelocityCommand(newForwVel, 0, newRotVel, newVertVel);
}

void FourDOFPropulsionController::transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform)
{
	geometry_msgs::PointStamped pointIn;
	
	pointIn.header.stamp = transform.stamp_;
	pointIn.header.frame_id = "/world";
	pointIn.point.x = pointPath[currentPoint].getX();
	pointIn.point.y = pointPath[currentPoint].getY();
	pointIn.point.z = pointPath[currentPoint].getZ();

	listener.transformPoint("/" + vehicleName, pointIn, pointOut);
}

bool FourDOFPropulsionController::isAtPoint(tf::Transform& location, tf::Vector3& point)
{
	double xDifference = fabs(location.getOrigin().getX() - point.getX());
	double yDifference = fabs(location.getOrigin().getY() - point.getY());
	double zDifference = fabs(location.getOrigin().getZ() - point.getZ());

	return zDifference <= verticalError && sqrt(yDifference * yDifference + xDifference * xDifference) <= lateralError;
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