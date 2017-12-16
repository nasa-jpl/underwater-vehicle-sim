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
}


void FourDOFPropulsion::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	linVelocity.setX(vel->linear.x);
	linVelocity.setY(vel->linear.y);
	linVelocity.setZ(vel->linear.z);

	rotVelocity.setZ(vel->angular.z);
}


tf::Transform FourDOFPropulsion::move(tf::StampedTransform currentLocation) 
{
	//Get the elapsed time since the last vehicle location update
	ros::Duration elapsedTime = (ros::Time::now() - currentLocation.stamp_);

	//Get the total linear movement in the vehicle frame
	tf::Vector3 totalLinMovement = linVelocity * elapsedTime.toSec(); 
	
	//rotate the total linear movement to be in the world frame
	totalLinMovement = totalLinMovement.rotate(currentLocation.getRotation().getAxis(), currentLocation.getRotation().getAngle());

	//apply the linear movement
	tf::Vector3 finalLinLocation = currentLocation.getOrigin() + (totalLinMovement * elapsedTime.toSec());


	//create a Quaternion to represent rotation using the axis of rotation and angle of rotation
	tf::Quaternion totalRotMovement(rotVelocity, rotVelocity.length() * elapsedTime.toSec());
	
	//Apply the rotation to the current rotation of the vehicle
	tf::Quaternion finalRotLocation = currentLocation.getRotation() * totalRotMovement;

	//return updated location
	return tf::Transform(finalRotLocation, finalLinLocation);
}