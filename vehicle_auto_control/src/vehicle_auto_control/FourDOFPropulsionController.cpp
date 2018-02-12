#include <math.h>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include "geometry_msgs/PointStamped.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "vehicle_auto_control/PointPathAction.h"

#include "underwater_vehicle_sim/VehicleData.h"

FourDOFPropulsionController::FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName, float loopHertz) :
	PropulsionController(controlNode, vehicleNode, vehicleName, loopHertz),
	targetHorzVelocity(0),
	targetRotVelocity(0),
	targetVertVelocity(0),
	lateralError(5.0),
	verticalError(1.0),
	latestSonarDepth(1000),
	minSeafloorDistance(5.0),
	pointPathServer(controlNode, "point_path", boost::bind(&FourDOFPropulsionController::executePointPath, this, _1, &pointPathServer), false)
{
	velocityPub = vehicleNode.advertise<geometry_msgs::Twist>(propModuleName + "/command_velocity", 1000);

	if(dataModuleName != "")
	{
		hasVehicleData = true;
		dataSub = vehicleNode.subscribe(dataModuleName + "/data", 1, &FourDOFPropulsionController::getVehicleData, this);
	}

	velocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
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
		double sonarDistance;
		try
		{
			listener.lookupTransform("/world", "/" + vehicleName,  
									 ros::Time(0), transform);

			if(currentPoint < pathPoints.size())
			{
				if(isAtPoint(transform, pathPoints[currentPoint], !goal->yoyo))
				{
					currentPoint++;
					if(currentPoint < pathPoints.size())
					{
					}
					
				}
			}
			

			if(currentPoint >= pathPoints.size())
			{
				ROS_INFO("Auto Controller: Final Point Reached");
			}
			
			feedback.currentPoint = currentPoint;
			feedback.goingUp = goingUp;
			as->publishFeedback(feedback);
			
			if(as->isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("Auto Controller: Action Preempted");
				as->setPreempted();
				break;
			}

			if(goal->yoyo)
			{
				if(transform.getOrigin().getZ() + verticalError > goal->upperDepth || transform.getOrigin().getZ() + verticalError >= 0)
				{
					goingUp = false;
				}
				else if(transform.getOrigin().getZ() - verticalError < goal->lowerDepth || latestSonarDepth <= verticalError)
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
					//Also account for distance off bottom
					newVertVel = scaleVerticalVelocity(transform, pathPoints[currentPoint]);
				}

				geometry_msgs::PointStamped pointOut;

				transformPointToVehicleFrame(pointOut, transform, pathPoints[currentPoint]);
				tf::Vector3 vehicleForward(1, 0, 0);
				tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
				tf::Vector3 cross = vehicleForward.cross(targetPoint);

				double angle = vehicleForward.angle(targetPoint);		

				//Get scaled rotational velocity
				newRotVel = scaleRotationalVelocity(angle, cross.getZ());

				//Set forward velocity
				newForwVel = scaleHorizontalVelocity(transform, pathPoints[currentPoint]);
			}

			//Create velocity command message and send it to the propulsion module
			sendVelocityCommand(newForwVel, 0, newRotVel, newVertVel);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		r.sleep();
	}

	if(currentPoint == pathPoints.size())
	{
		ROS_INFO("Auto Controller: Action Done, Succeeded");
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

double FourDOFPropulsionController::scaleHorizontalVelocity(tf::Transform& location, tf::Vector3& point)
{
	double xDifference = fabs(location.getOrigin().getX() - point.getX());
	double yDifference = fabs(location.getOrigin().getY() - point.getY());
	double xyError = sqrt(yDifference * yDifference + xDifference * xDifference);

	double horizontalScaleError = 50;

	if(xyError >= horizontalScaleError)
	{
		return targetHorzVelocity;
	}
	
	return targetHorzVelocity * (xyError / horizontalScaleError);
}

double FourDOFPropulsionController::scaleVerticalVelocity(tf::Transform& location, tf::Vector3& point)
{

	double zDifference = fabs(location.getOrigin().getZ() - point.getZ());

	double verticalScaleError = 15;

	int sign = 0;
	if(point.getZ() >= location.getOrigin().getZ())
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	if(zDifference >= verticalScaleError)
	{
		return targetVertVelocity * sign;
	}
	
	return targetVertVelocity * (zDifference / verticalScaleError) * sign;
}

double FourDOFPropulsionController::scaleRotationalVelocity(double angleError, double crossZ)
{
	double angleErrorScale = 0.523599; //30 degrees

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