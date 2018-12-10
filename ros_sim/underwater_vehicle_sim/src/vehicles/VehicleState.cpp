#include "vehicles/VehicleState.h"

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "model_server/GetModelData.h"

#define SECONDS_IN_DAY 86400

VehicleState::VehicleState(ros::NodeHandle& nh) :
	angularVelocity(0,0,0),
	linearVelocity(0,0,0),
	powerCapacity(0),
	dataCapacity(0)
{
	modelClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");
}

VehicleState::VehicleState(VehicleState&& other)
	: position(std::move(other.position)), 
      linearVelocity(std::move(other.linearVelocity)),
      rotation(std::move(other.rotation)),
      angularVelocity(std::move(other.angularVelocity)),
      powerCapacity(std::move(other.powerCapacity)),
      dataCapacity(std::move(other.dataCapacity)),
      modelClient(std::move(other.modelClient))
{}

void VehicleState::updatePose(const ros::Time currentTime, const ros::Duration deltaTime)
{
    //Get the total linear movement in the vehicle frame
	tf::Vector3 totalLinMovement = linearVelocity * deltaTime.toSec();

	//rotate the total linear movement to be in the world frame
	totalLinMovement = totalLinMovement.rotate(rotation.getAxis(), rotation.getAngle());

	//apply the linear movement
	position += totalLinMovement;

    //create a Quaternion to represent rotation using the axis of rotation and angle of rotation
	tf::Quaternion totalRotMovement;

    //Z rotation is negative because heading increases clockwise
	totalRotMovement.setRPY(angularVelocity.getX() * deltaTime.toSec(),
							angularVelocity.getY() * deltaTime.toSec(),
							angularVelocity.getZ() * deltaTime.toSec());
	
	//Apply the rotation to the current rotation of the vehicle
	rotation *= totalRotMovement;

	//prevent position from leaving the top of the model
	if(position.getZ() > 0)
	{
		position.setZ(0);
	}

	if(modelClient.exists())
	{
		model_server::GetModelData srv;

		srv.request.x = position.getX();
		srv.request.y = position.getY();
		srv.request.h = position.getZ();
		srv.request.time = currentTime.toSec() / SECONDS_IN_DAY; //convert from seconds to days

		bool success = modelClient.call(srv);

		if(success && -srv.response.depth + 0.1 > position.getZ())
		{
			//if vehicle is trying to go below the bottom of the ocean model then set its z position to above the ocean floor.
			position.setZ(-srv.response.depth + 0.1);
		}
	}
}

tf::Vector3 VehicleState::getPosition() const
{
    return position;
}

tf::Vector3 VehicleState::getLinearVelocity() const
{
    return linearVelocity;
}

tf::Quaternion VehicleState::getRotation() const
{
    return rotation;
}

tf::Vector3 VehicleState::getAngularVelocity() const
{
    return angularVelocity;
}

void VehicleState::setPosition(const tf::Vector3 position)
{
	this->position = position;
}

void VehicleState::setRotation(const tf::Quaternion rotation)
{
	this->rotation = rotation;
}

void VehicleState::setLinearVelocity(const tf::Vector3 velocity)
{
    linearVelocity = velocity;
}

void VehicleState::setAngularVelocity(const tf::Vector3 velocity)
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