#include "vehicles/VehicleState.h"

#include "ros/ros.h"

#include "model_server/GetModelData.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

VehicleState::VehicleState() :
	angularVelocity(0,0,0),
	linearVelocity(0,0,0),
	powerCapacity(0),
	dataCapacity(0)
{

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	
	geometry_msgs::TransformStamped transformToNED;
	geometry_msgs::TransformStamped transformToENU;

	while(!tfBuffer.canTransform("world", "world_ned", ros::Time(0), ros::Duration(100)));

	try
	{
    	transformToENU = tfBuffer.lookupTransform("world", "world_ned",
                               		ros::Time(0));

		transformToNED = tfBuffer.lookupTransform("world_ned", "world",
                               		ros::Time(0));

		tf2::fromMsg(transformToENU.transform, NEDtoENU);
		tf2::fromMsg(transformToNED.transform, ENUtoNED);
    }
    catch (tf2::TransformException &ex)
	{
    	ROS_ERROR("%s",ex.what());
    }
}

VehicleState::VehicleState(VehicleState&& other)
	: position(std::move(other.position)), 
      linearVelocity(std::move(other.linearVelocity)),
      rotation(std::move(other.rotation)),
      angularVelocity(std::move(other.angularVelocity)),
      powerCapacity(std::move(other.powerCapacity)),
      dataCapacity(std::move(other.dataCapacity))
{}

void VehicleState::updatePose(const ros::Time currentTime, const ros::Duration deltaTime)
{
    //Get the total linear movement in the vehicle frame
	tf2::Vector3 totalLinMovement = linearVelocity * deltaTime.toSec();

	//rotate the total linear movement to be in the world frame
	totalLinMovement = totalLinMovement.rotate(rotation.getAxis(), rotation.getAngle());

	//apply the linear movement
	position += totalLinMovement;

	if(angularVelocity.length() != 0)
	{
		//Apply the angular velocity * delta time to the current rotation of the vehicle
		tf2::Quaternion deltaRotation(angularVelocity.normalized(), angularVelocity.length() * deltaTime.toSec());
		rotation = deltaRotation * rotation;
	}
						  
	//prevent position from leaving the top of the model
	if(position.getZ() < 0)
	{
		position.setZ(0);
	}
}

bool VehicleState::seafloorCollision(ModelData& data)
{
	tf2::Vector3 enuPosition = getPositionENU();

	if(!std::isnan(data.depth) && 
	   -data.depth + 0.1 > enuPosition.getZ())
	{
		//if vehicle is trying to go below the bottom of the ocean model then set its z position to above the ocean floor.
		enuPosition.setZ(-data.depth + 0.1);
		setPositionENU(enuPosition);
		return true;
	}
	return false;
}

void VehicleState::evectByCurrents(ModelData& data, const ros::Duration deltaTime)
{
	if(!std::isnan(data.u) &&
	   !std::isnan(data.v) &&
	   !std::isnan(data.w))
	{
		//Get the total linear movement due to currents
		//v is north (x in NED coordinate frame)
		//u is east (y in NED coordinate frame)
		//w is upward (-z in NED coordinate frame)
		tf2::Vector3 currentMovement(data.v, data.u, -data.w);

		//apply the movement due to currents
		position += currentMovement * deltaTime.toSec();
	}
}

tf2::Vector3 VehicleState::getPositionNED() const
{
    return position;
}

tf2::Vector3 VehicleState::getPositionENU() const
{
	tf2::Vector3 enuPos = NEDtoENU(position);
	return enuPos;
}

tf2::Vector3 VehicleState::getLinearVelocityNED() const
{
    return linearVelocity;
}

tf2::Quaternion VehicleState::getRotationNED() const
{
    return rotation;
}

tf2::Quaternion VehicleState::getRotationENU() const
{
	return NEDtoENU * rotation;
}

tf2::Vector3 VehicleState::getAngularVelocityNED() const
{
    return angularVelocity;
}

void VehicleState::setPositionNED(const tf2::Vector3 position)
{
	this->position = position;
}

void VehicleState::setPositionENU(const tf2::Vector3 position)
{
	this->position = ENUtoNED(position);
}

void VehicleState::setRotationNED(const tf2::Quaternion rotation)
{
	this->rotation = rotation;
}

void VehicleState::setRotationENU(const tf2::Quaternion rotation)
{
	this->rotation = ENUtoNED * rotation;
}

void VehicleState::setLinearVelocityNED(const tf2::Vector3 velocity)
{
    linearVelocity = velocity;
}

void VehicleState::setAngularVelocityNED(const tf2::Vector3 velocity)
{
    angularVelocity = velocity;
}

double VehicleState::getPowerCapacity()
{
	return powerCapacity;
}
    
double VehicleState::getDataCapacity()
{
	return dataCapacity;
}