#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include "geometry_msgs/PointStamped.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "vehicle_auto_control/PointPathAction.h"


FourDOFPropulsionController::FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string vehicleName, float loopHertz) :
	PropulsionController(controlNode, vehicleNode, vehicleName, loopHertz),
	targetHorzVelocity(0),
	targetRotVelocity(0),
	targetVertVelocity(0),
	lateralError(1.0),
	verticalError(0.25),
	rotationalError(0.0174533),
	pointPathServer(controlNode, "point_path", boost::bind(&FourDOFPropulsionController::executePointPath, this, _1, &pointPathServer), false)
{
	velocityPub = vehicleNode.advertise<geometry_msgs::Twist>("command_velocity", 1000);

	velocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
	pointPathServer.start();
}

void FourDOFPropulsionController::getTargetVelocityCommand(const vehicle_auto_control::Velocity vel)
{
	targetHorzVelocity = fabs(vel.horizontalVelocity);
	targetRotVelocity = fabs(vel.rotationalVelocity);
	targetVertVelocity = fabs(vel.verticalVelocity);
}

void FourDOFPropulsionController::executePointPath(const vehicle_auto_control::PointPathGoalConstPtr& goal, 
												   actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction>* as)
{
	//Feedback and Results for the action
	vehicle_auto_control::PointPathFeedback feedback;
    vehicle_auto_control::PointPathResult result;

	//Rate at which to run the control loop
	ros::Rate r(loopHertz);

	std::vector<tf::Vector3> pathPoints;
	for(auto point: goal->points)
	{
		pathPoints.emplace_back(point.x, point.y, point.z);
	}

	double newVertVel = 0;
	double newRotVel = 0;
	double newForwVel = 0;

	unsigned int currentPoint = 0;
	bool goingUp = true;

	while(currentPoint < pathPoints.size() && ros::ok())
	{
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

		if(currentPoint < pathPoints.size())
		{
			if(isAtPoint(transform, pathPoints[currentPoint], !goal->yoyo))
			{
				currentPoint++;
			}
		}

		feedback.currentPoint = currentPoint;
		as->publishFeedback(feedback);
		
		if(as->isPreemptRequested() || !ros::ok())
		{
			as->setPreempted();
			break;
		}

		if(goal->yoyo)
		{
			if(transform.getOrigin().getZ() + verticalError > goal->upperDepth)
			{
				goingUp = false;
			}
			else if(transform.getOrigin().getZ() - verticalError < goal->lowerDepth)
			{
				goingUp = true;
			}
		}

		if(currentPoint < pathPoints.size())
		{
			//Set vertical
			if(goal->yoyo)
			{
				if(goingUp)
				{
					newVertVel = targetVertVelocity;	
				}
				else
				{
					newVertVel = -targetVertVelocity;
				}
			}
			else
			{
				if(pathPoints[currentPoint].getZ() - transform.getOrigin().getZ()  > verticalError)
				{
					newVertVel = targetVertVelocity;
				}
				else if(pathPoints[currentPoint].getZ() - transform.getOrigin().getZ() < -verticalError)
				{
					newVertVel = -targetVertVelocity;
				}
			}

			geometry_msgs::PointStamped pointOut;
			transformPointToVehicleFrame(pointOut, transform, pathPoints[currentPoint]);

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

		r.sleep();
	}

	if(currentPoint == pathPoints.size())
	{
		result.totalPoints = currentPoint;
		as->setSucceeded(result);
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
	double xDifference = fabs(location.getOrigin().getX() - point.getX());
	double yDifference = fabs(location.getOrigin().getY() - point.getY());
	double zDifference = fabs(location.getOrigin().getZ() - point.getZ());

	return (!useZ || zDifference <= verticalError) && sqrt(yDifference * yDifference + xDifference * xDifference) <= lateralError;
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