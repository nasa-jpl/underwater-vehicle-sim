#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <vector>
#include <memory>

#include "ros/ros.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "model_server/GetModelData.h"
#include "ocean_models/model_interface/ModelData.h"

/**
 * Class used to represent a vehicle in the simulation
 * Vehicle front is the position x-axis in the vehicle frame
 * Vehicle rotation follows the standard right-hand rule with 0 degrees on the positive x-axis in the world frame
 * Velocities are in the body frame of the vehicle
 */
class VehicleState
{
public:
	VehicleState();
	VehicleState(VehicleState&& other);

    /**
     * Update the vehicle state based on velocities
     * @currentTime The time of the update
     * @deltaTime The time since the last update
     */
    void updatePose(const ros::Time currentTime, const ros::Duration deltaTime);

    /**
     * Update the vehicle state if it is below the seafloor
     * @data Model data containing depth at the current location
     */
    bool seafloorCollision(ocean_models::ModelData& data);

    /**
    * Update the most recent model data for the vehicle
    * @data Model data containing currents are the current location and time
    */
    void updateModelData(ocean_models::ModelData& data);

    /**
     * Get the position of the vehicle in the NED frame.
     * This frame is used to define the vehicle state
     */
    tf2::Vector3 getPositionNED() const;

     /**
     * Get the position of the vehicle in the ENU frame.
     * This frame is used when interfacing with models
     */
    tf2::Vector3 getPositionENU() const;

    /**
     * Get the rotation of the vehicle in the NED frame.
     * This frame is used to define the vehicle state
     */
    tf2::Quaternion getRotationNED() const;

    /**
     * Get the rotation of the vehicle in the ENU frame.
     * This frame is used when interfacing with models
     */
    tf2::Quaternion getRotationENU() const;

    /**
     * Set the position of the vehicle in the NED frame
     * This frame is used to define the vehicle state
     */
    void setPositionNED(const tf2::Vector3 position);

    /**
     * Set the position of the vehicle in the ENU frame
     * This frame is used when interfacing with models
     */
    void setPositionENU(const tf2::Vector3 position);

    /**
     * Set the rotation of the vehicle in the NED frame
     * This frame is used to define the vehicle state
     */
    void setRotationNED(const tf2::Quaternion rotation);

    /**
     * Set the rotation of the vehicle in the ENU frame
     * This frame is used when interfacing with models
     */
    void setRotationENU(const tf2::Quaternion rotation);

    /**
    * Returns the linear velocity of the vehicle in the NED coordinate frame
    * @param withCurrents When true this will return the linear velocity with 
                          currents added. Otherwise without currents added.
    */
    tf2::Vector3 getLinearVelocityNED(bool withCurrents) const;

    tf2::Vector3 getAngularVelocityNED() const;
    void setLinearVelocityNED(const tf2::Vector3 velocity);
    void setAngularVelocityNED(const tf2::Vector3 velocity);
    	
    double getPowerCapacity();
    double getDataCapacity();
private:
    tf2::Vector3 modelCurrentsToBodyFrame() const;
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
    * Stores the latest model data at the vehicles location
    */
    ocean_models::ModelData latestModelData;

    tf2::Transform ENUtoNED;
    tf2::Transform NEDtoENU;
};

#endif
