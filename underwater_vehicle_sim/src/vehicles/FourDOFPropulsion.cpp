#include "ros/ros.h"

#include "vehicles/FourDOFPropulsion.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

FourDOFPropulsion::FourDOFPropulsion(std::string name, ros::NodeHandle& parentNH) :
	PropulsionModule(name, parentNH)
{
	nh.getParam("max_linear_velocity", maxLinVelocity);
	nh.getParam("max_rotate_velocity", maxRotVelocity);


	nh.subscribe("command_velocity", 1, &FourDOFPropulsion::commandVelocityCallback, this);

	rotVelocity.setX(0);
	rotVelocity.setY(0);
	rotVelocity.setZ(0);

	linVelocity.setX(0);
	linVelocity.setY(0);
	linVelocity.setZ(0);
}


void FourDOFPropulsion::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	linVelocity.setX(vel->linear.x);
	linVelocity.setY(vel->linear.y);
	linVelocity.setZ(vel->linear.z);

	rotVelocity.setZ(vel->angular.z);
}


void FourDOFPropulsion::move(ros::Time& lastTime, tf::Quaternion& rotation, tf::Vector3& position) 
{
	//Get the elapsed time since the last vehicle location update
	ros::Duration elapsedTime = (ros::Time::now() - lastTime);

	//Get the total linear movement in the vehicle frame
	tf::Vector3 totalLinMovement = linVelocity * elapsedTime.toSec(); 
	
	//rotate the total linear movement to be in the world frame
	totalLinMovement = totalLinMovement.rotate(rotation.getAxis(), rotation.getAngle());

	//apply the linear movement
	position += (totalLinMovement * elapsedTime.toSec());

	//create a Quaternion to represent rotation using the axis of rotation and angle of rotation
	tf::Quaternion totalRotMovement;

	totalRotMovement.setRPY(rotVelocity.getX() * elapsedTime.toSec(),
							rotVelocity.getY() * elapsedTime.toSec(),
							rotVelocity.getZ() * elapsedTime.toSec());
	
	//Apply the rotation to the current rotation of the vehicle
	rotation *= totalRotMovement;
}