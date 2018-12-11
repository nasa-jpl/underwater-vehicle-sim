#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <vector>
#include <memory>

#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include "model_server/GetModelData.h"

/**
 * Class used to represent a vehicle in the simulation
 * Vehicle front is the position x-axis in the vehicle frame
 * Vehicle rotation follows the standard right-hand rule with 0 degrees on the positive x-axis in the world frame
 */
class VehicleState
{
public:
	VehicleState(ros::NodeHandle& nh);
	VehicleState(VehicleState&& other);

    void updatePose(const ros::Time currentTime, const ros::Duration deltaTime);

    tf2::Vector3 getPosition() const;
    tf2::Vector3 getLinearVelocity() const;

    tf2::Quaternion getRotation() const;
    tf2::Vector3 getAngularVelocity() const;

    void setPosition(const tf2::Vector3 position);
    void setRotation(const tf2::Quaternion rotation);
    void setLinearVelocity(const tf2::Vector3 velocity);
    void setAngularVelocity(const tf2::Vector3 velocity);
    	
    double getPowerCapacity();
    double getDataCapacity();
private:
	/**
	* Current vehicle position espressed as a tf vector
	*/
	tf2::Vector3 position;

    /**
     * Current linear velocity of the vehicle in body frame in m/s
    **/
    tf2::Vector3 linearVelocity;

    /**
	* Current vehicle rotation expressed as a tf quaternion
	*/
	tf2::Quaternion rotation;

    /**
     * Current rotational velocity of the vehicle in rad/s
    **/
    tf2::Vector3 angularVelocity;

	/**
	* Tracks power remaining for this vehicle
	*/
	double powerCapacity; 
	
	/**
	* Tracks data storage remaining for this vehicle
	*/
	double dataCapacity;

    /**
    * Model client used to prevent the vehicle from clipping through the seafloor or water surface
    */
    ros::ServiceClient modelClient;
};

#endif
