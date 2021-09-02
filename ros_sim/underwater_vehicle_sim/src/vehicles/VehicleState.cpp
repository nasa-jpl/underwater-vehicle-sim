#include "vehicles/VehicleState.h"

#include "ros/ros.h"

#include "model_server/GetModelData.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace ocean_model_interfaces;

VehicleState::VehicleState() :
    linearVelocity(0,0,0),
    angularVelocity(0,0,0),
    powerCapacity(0),
    dataCapacity(0)
{
    latestModelData.u = 0;
    latestModelData.v = 0;
    latestModelData.w = 0;

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
      dataCapacity(std::move(other.dataCapacity)),
      latestModelData(std::move(other.latestModelData))
{}

void VehicleState::updatePose(const ros::Time currentTime, const ros::Duration deltaTime)
{
    //Get the total linear movement in the body frame
    tf2::Vector3 totalLinMovement = (linearVelocity + modelCurrentsToBodyFrame()) * deltaTime.toSec();

    //rotate the total linear movement from the body frame into the world frame
    totalLinMovement = totalLinMovement.rotate(rotation.getAxis(), rotation.getAngle());

    //apply the linear movement
    position += totalLinMovement;

    if(angularVelocity.length() != 0)
    {
        //rotate from body frame to world frame
        tf2::Vector3 worldFrameAngularVelocity = angularVelocity.rotate(rotation.getAxis(), rotation.getAngle());

        //Apply the world frame angular velocity * delta time to the current rotation of the vehicle
        tf2::Quaternion deltaRotation(worldFrameAngularVelocity.normalized(), 
                                      worldFrameAngularVelocity.length() * deltaTime.toSec());
        
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

void VehicleState::updateModelData(ocean_model_interfaces::ModelData& data)
{
    if(!std::isnan(data.u) &&
       !std::isnan(data.v) &&
       !std::isnan(data.w))
    {
        latestModelData = data;
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

tf2::Vector3 VehicleState::getLinearVelocityNED(bool withCurrents) const
{
    if(withCurrents) {
        return linearVelocity + modelCurrentsToBodyFrame();
    }
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

tf2::Vector3 VehicleState::modelCurrentsToBodyFrame() const
{
    tf2::Vector3 currents(latestModelData.u, latestModelData.v, latestModelData.w);

    //Model is in ENU so convert to NED before rotating to body frame
    currents = ENUtoNED * currents;
    currents = currents.rotate(-rotation.getAxis(), rotation.getAngle());

    return currents;
}
