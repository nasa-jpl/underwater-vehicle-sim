#include "ros_underwater_sim_utilities/VehiclePose.h"

#include <cmath>
#include <iostream>

VehiclePose::VehiclePose() :
    position(0,0,0),
    linearVelocity(0,0,0),
    angularVelocity(0,0,0)
{
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    orientation = Eigen::Quaterniond(m);
}

VehiclePose::VehiclePose(Eigen::Vector3d position) :
    position(position),
    linearVelocity(0,0,0),
    angularVelocity(0,0,0)
{
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    orientation = Eigen::Quaterniond(m);
}

VehiclePose::VehiclePose(Eigen::Vector3d position, Eigen::Quaterniond orientation) :
    position(position),
    orientation(orientation),
    linearVelocity(0,0,0),
    angularVelocity(0,0,0)
{}

Eigen::Vector3d VehiclePose::getPosition() const
{
    return position;
}

Eigen::Quaterniond VehiclePose::getOrientation() const
{
    return orientation;
}

Eigen::Matrix<double, 3, 1> VehiclePose::getRPY() const {
    return orientation.toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Matrix<double,6,6> VehiclePose::getPoseCovariance() const
{
    return poseCovariance;
}

Eigen::Vector3d VehiclePose::getLinearVelocity() const
{
    return linearVelocity;
}

Eigen::Vector3d VehiclePose::getAngularVelocity() const
{
    return angularVelocity;
}

Eigen::Matrix<double,6,6> VehiclePose::getTwistCovariance() const
{
    return twistCovariance;
}

void VehiclePose::setPosition(Eigen::Vector3d position)
{
    this->position = position;
}

void VehiclePose::setOrientation(Eigen::Quaterniond orientation)
{
    this->orientation = orientation;
}

void VehiclePose::setPoseCovariance(Eigen::Matrix<double,6,6> poseCovariance)
{
    this->poseCovariance = poseCovariance;
}

void VehiclePose::setLinearVelocity(Eigen::Vector3d linearVelocity)
{
    this->linearVelocity = linearVelocity;
}

void VehiclePose::setAngularVelocity(Eigen::Vector3d angularVelocity)
{
    this->angularVelocity = angularVelocity;
}

void VehiclePose::setTwistCovariance(Eigen::Matrix<double,6,6> twistCovariance)
{
    this->twistCovariance = twistCovariance;
}

bool VehiclePose::operator==(const VehiclePose& rhs)
{
    return fabs(position[0] - rhs.position[0]) <= 0.000001 &&
           fabs(position[1] - rhs.position[1]) <= 0.000001 &&
           fabs(position[2] - rhs.position[2]) <= 0.000001 &&
           fabs(orientation.dot(rhs.orientation) - 1) <= 0.000001 &&
           fabs(linearVelocity[0] - rhs.linearVelocity[0]) <= 0.000001 &&
           fabs(linearVelocity[1] - rhs.linearVelocity[1]) <= 0.000001 &&
           fabs(linearVelocity[2] - rhs.linearVelocity[2]) <= 0.000001 &&
           fabs(angularVelocity[0] - rhs.angularVelocity[0]) <= 0.000001 &&
           fabs(angularVelocity[1] - rhs.angularVelocity[1]) <= 0.000001 &&
           fabs(angularVelocity[2] - rhs.angularVelocity[2]) <= 0.000001;
}