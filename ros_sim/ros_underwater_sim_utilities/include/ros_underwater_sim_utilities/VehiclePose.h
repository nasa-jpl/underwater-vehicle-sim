#ifndef VEHICLE_POSE_H
#define VEHICLE_POSE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


/**
* Describes the pose of the vehicle. The position and orientation of the vehicle as well as the velocities are included.
* Position and orientation are in world frame. Velocities are in NED body frame.
*/
class VehiclePose
{
public:
    VehiclePose();
    VehiclePose(Eigen::Vector3d position);
    VehiclePose(Eigen::Vector3d position, Eigen::Quaterniond orientation);
    ~VehiclePose() {}

    /**
    * Get the position as a 3d vector.
    */
    Eigen::Vector3d getPosition() const;

    /**
    * Get the orientation as a quaterion.
    */
    Eigen::Quaterniond getOrientation() const;

    /**
    * Get the individual Roll, Pitch, and Yaw angles
    */
    Eigen::Matrix<double, 3, 1>  getRPY() const;

    /**
    * Get the covariance of the vehicles pose (Position and Orientation)
    */
    Eigen::Matrix<double,6,6> getPoseCovariance() const;

    /**
    * Get the linear velocity of the vehicle.
    */
    Eigen::Vector3d getLinearVelocity() const;

    /**
    * Get the angular velocity of the vehicle.
    */
    Eigen::Vector3d getAngularVelocity() const;

    /**
    * Get the covariance of the linear and angular velocities.
    */
    Eigen::Matrix<double,6,6> getTwistCovariance() const;

    void setPosition(Eigen::Vector3d position);
    void setOrientation(Eigen::Quaterniond orientation);
    void setPoseCovariance(Eigen::Matrix<double,6,6> poseCovariance);
    
    void setLinearVelocity(Eigen::Vector3d linearVelocity);
    void setAngularVelocity(Eigen::Vector3d angularVelocity);
    void setTwistCovariance(Eigen::Matrix<double,6,6> twistCovariance);

    bool operator== (const VehiclePose& rhs);
private:

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Matrix<double,6,6> poseCovariance;

    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d angularVelocity;
    Eigen::Matrix<double,6,6> twistCovariance;
};

bool operator== (const VehiclePose& lhs, const VehiclePose& rhs);

#endif