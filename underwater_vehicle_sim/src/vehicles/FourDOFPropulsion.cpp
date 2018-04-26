#include "ros/ros.h"

#include "vehicles/FourDOFPropulsion.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#define SECONDS_IN_DAY 86400


FourDOFPropulsion::FourDOFPropulsion(std::string name, ros::NodeHandle& parentNH) :
	PropulsionModule(name, "FourDOFPropulsion", parentNH)
{
	nh.getParam("max_linear_velocity", maxLinVelocity);
	nh.getParam("max_rotate_velocity", maxRotVelocity);

	modelClient = nh.serviceClient<model_server::GetModelData>("get_model_data");
	commandVelocitySub = nh.subscribe("command_velocity", 1, &FourDOFPropulsion::commandVelocityCallback, this);

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


void FourDOFPropulsion::move(ros::Time& lastTime, tf::Quaternion& rotation, tf::Vector3& position, double& powerCapacity, double& dataCapacity) 
{
	//Get the elapsed time since the last vehicle location update
	ros::Duration elapsedTime = (ros::Time::now() - lastTime);
	lastTime = ros::Time::now();

	//Get the total linear movement in the vehicle frame
	tf::Vector3 totalLinMovement = linVelocity * elapsedTime.toSec(); 
	
	//rotate the total linear movement to be in the world frame
	totalLinMovement = totalLinMovement.rotate(rotation.getAxis(), rotation.getAngle());

	//apply the linear movement
	position += totalLinMovement;

	//create a Quaternion to represent rotation using the axis of rotation and angle of rotation
	tf::Quaternion totalRotMovement;

	totalRotMovement.setRPY(rotVelocity.getX() * elapsedTime.toSec(),
							rotVelocity.getY() * elapsedTime.toSec(),
							rotVelocity.getZ() * elapsedTime.toSec());
	
	//Apply the rotation to the current rotation of the vehicle
	rotation *= totalRotMovement;



	model_server::GetModelData srv;

	srv.request.x = position.getX();
	srv.request.y = position.getY();
	srv.request.h = position.getZ();
	srv.request.time = lastTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days

	//prevent position from leaving the top of the model
	if(position.getZ() > 0)
	{
		position.setZ(0);
	}

	if(modelClient.exists())
	{
		bool success = modelClient.call(srv);

		if(success && -srv.response.depth + 0.1 > position.getZ())
		{
			//if vehicle is trying to go below the bottom of the ocean model then set its z position to above the ocean floor.
			position.setZ(-srv.response.depth + 0.1);
		}
	}
	//Use power PLACEHOLDER
	powerCapacity -= 0.01 * elapsedTime.toSec();
}