#include <algorithm>
#include <limits>

#include "ros/ros.h"
#include "tf2/LinearMath/Vector3.h"

#include "vehicles/FourDOFPropulsion.h"
#include "model_server/GetModelData.h"

#define SECONDS_IN_DAY 86400

FourDOFPropulsion::FourDOFPropulsion(std::string name, VehicleState& vehicleState, ros::NodeHandle& parentNH) :
	PropulsionModule(name, "FourDOFPropulsion", vehicleState, parentNH)
{
    if(nh.hasParam("max_linear_velocity"))
    {
        nh.getParam("max_linear_velocity", maxLinVelocity);
        if(maxLinVelocity < 0)
        {
            maxLinVelocity = 0;
        }
    }
    else
    {
        maxLinVelocity = std::numeric_limits<double>::max();
    }

    if(nh.hasParam("max_vertical_velocity"))
    {
        nh.getParam("max_vertical_velocity", maxVertVelocity);
        if(maxVertVelocity < 0)
        {
            maxVertVelocity = 0;
        }
    }
    else
    {
        maxVertVelocity = std::numeric_limits<double>::max();
    }

    if(nh.hasParam("max_angular_velocity"))
    {
        nh.getParam("max_angular_velocity", maxRotVelocity);
        if(maxRotVelocity < 0)
        {
            maxRotVelocity = 0;
        }
    }
    else
    {
        maxRotVelocity = std::numeric_limits<double>::max();
    }

	commandVelocitySub = nh.subscribe("command_velocity", 1, &FourDOFPropulsion::commandVelocityCallback, this);
    vehicleState.setLinearVelocity(tf2::Vector3(0,0,0));
    vehicleState.setAngularVelocity(tf2::Vector3(0,0,0));
}


void FourDOFPropulsion::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    tf2::Vector3 updatedLinearVelocity;
    tf2::Vector3 updatedAngularVelocity;

    if(std::isfinite(vel->linear.x))
    {
        if((fabs(vel->linear.x) < maxLinVelocity))
        {
            updatedLinearVelocity.setX(vel->linear.x);
        }
        else
        {
            if(vel->linear.x >= 0)
            {
                updatedLinearVelocity.setX(maxLinVelocity);
            }
            else
            {
                updatedLinearVelocity.setX(-maxLinVelocity);
            }
        }
    }

    if(std::isfinite(vel->linear.y))
    {
        if((fabs(vel->linear.y) < maxLinVelocity))
        {
            updatedLinearVelocity.setY(vel->linear.y);
        }
        else
        {
            if(vel->linear.y >= 0)
            {
                updatedLinearVelocity.setY(maxLinVelocity);
            }
            else
            {
                updatedLinearVelocity.setY(-maxLinVelocity);
            }
        }
    }

    if(std::isfinite(vel->linear.z))
    {
        if((fabs(vel->linear.z) < maxVertVelocity))
        {
            updatedLinearVelocity.setZ(vel->linear.z);
        }
        else
        {
            if(vel->linear.z >= 0)
            {
                updatedLinearVelocity.setZ(maxVertVelocity);
            }
            else
            {
                updatedLinearVelocity.setZ(-maxVertVelocity);
            }
        }
    }

    if(std::isfinite(vel->angular.z))
    {
        if((fabs(vel->angular.z) < maxRotVelocity))
        {
            updatedAngularVelocity.setZ(vel->angular.z);
        }
        else
        {
            if(vel->angular.z >= 0)
            {
                updatedAngularVelocity.setZ(maxRotVelocity);
            }
            else
            {
                updatedAngularVelocity.setZ(-maxRotVelocity);
            }
        }  
    }

    vehicleState.setLinearVelocity(updatedLinearVelocity);
    vehicleState.setAngularVelocity(updatedAngularVelocity);
}