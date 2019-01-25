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

    if(nh.hasParam("forward_thruster_thrust") && 
       nh.hasParam("forward_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nh.getParam("forward_thruster_thrust", thrust);
        nh.getParam("forward_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {

            if(thrust.size() == 0)
            {
                ROS_WARN("%s/forward_thruster_thrust and %s/forward_thruster_velocity are empty.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < thrust.size(); i++)
                {
                    points.push_back({thrust[i], velocity[i]});
                }
                forwardThrusterFunc = LinearPiecewise(points);
            }
        }
        else
        {
            ROS_WARN("%s/forward_thruster_thrust and %s/forward_thruster_velocity have differing length.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/forward_thruster_thrust or %s/forward_thruster_velocity does not exist.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
    }

    if(nh.hasParam("lateral_thruster_thrust") && 
       nh.hasParam("lateral_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nh.getParam("lateral_thruster_thrust", thrust);
        nh.getParam("lateral_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {
            if(thrust.size() == 0)
            {
                ROS_WARN("%s/lateral_thruster_thrust and %s/lateral_thruster_velocity are empty.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < thrust.size(); i++)
                {
                    points.push_back({thrust[i], velocity[i]});
                }
                lateralThrusterFunc = LinearPiecewise(points);
            }
        }
        else
        {
            ROS_WARN("%s/lateral_thruster_thrust and %s/lateral_thruster_velocity have differing length.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/lateral_thruster_thrust or %s/lateral_thruster_velocity does not exist.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
    }

    if(nh.hasParam("vertical_thruster_thrust") && 
       nh.hasParam("vertical_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nh.getParam("vertical_thruster_thrust", thrust);
        nh.getParam("vertical_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {
            if(thrust.size() == 0)
            {
                ROS_WARN("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity are empty.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < thrust.size(); i++)
                {
                    points.push_back({thrust[i], velocity[i]});
                }
                verticalThrusterFunc = LinearPiecewise(points);
            }
        }
        else
        {
            ROS_WARN("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity have differing length.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/vertical_thruster_thrust or %s/vertical_thruster_velocity does not exist.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
    }

    if(nh.hasParam("rudder_angle") && 
       nh.hasParam("rudder_velocity"))
    {
        std::vector<double> angle;
        std::vector<double> velocity;
        nh.getParam("rudder_angle", angle);
        nh.getParam("rudder_velocity", velocity);

        if(angle.size() == velocity.size())
        {
            if(angle.size() == 0)
            {
                ROS_WARN("%s/rudder_angle and %s/rudder_velocity are empty.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < angle.size(); i++)
                {
                    points.push_back({angle[i], velocity[i]});
                }
                rudderFunc = LinearPiecewise(points);
            }
        }
        else
        {
            ROS_WARN("%s/rudder_angle and %s/rudder_velocity have differing length.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/rudder_angle or %s/rudder_velocity does not exist.  Default values will be used.", nh.getNamespace().c_str(), nh.getNamespace().c_str());
    }

    forwardThrusterSub = nh.subscribe("command_forward_thruster", 1, &FourDOFPropulsion::forwardThrusterCallback, this);
	lateralThrusterSub = nh.subscribe("command_lateral_thruster", 1, &FourDOFPropulsion::lateralThrusterCallback, this);
    verticalThrusterSub = nh.subscribe("command_vertical_thruster", 1, &FourDOFPropulsion::verticalThrusterCallback, this);
	rudderSub = nh.subscribe("command_rudder", 1, &FourDOFPropulsion::rudderCallback, this);

    vehicleState.setLinearVelocity(tf2::Vector3(0,0,0));
    vehicleState.setAngularVelocity(tf2::Vector3(0,0,0));
}

void FourDOFPropulsion::forwardThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    tf2::Vector3 updatedLinearVelocity = vehicleState.getLinearVelocity();
    if(std::isfinite(val->data))
    {
        LinearPiecewise::Point vel = forwardThrusterFunc.getY(val->data);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setX(vel.y);
        }
    }

    vehicleState.setLinearVelocity(updatedLinearVelocity);
}

void FourDOFPropulsion::lateralThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    tf2::Vector3 updatedLinearVelocity = vehicleState.getLinearVelocity();

    if(std::isfinite(val->data))
    {
        LinearPiecewise::Point vel = lateralThrusterFunc.getY(val->data);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setY(vel.y);
        }
    }

    vehicleState.setLinearVelocity(updatedLinearVelocity);
}

void FourDOFPropulsion::verticalThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    tf2::Vector3 updatedLinearVelocity = vehicleState.getLinearVelocity();

    if(std::isfinite(val->data))
    {
        LinearPiecewise::Point vel = verticalThrusterFunc.getY(val->data);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setZ(vel.y);
        }
    }

    vehicleState.setLinearVelocity(updatedLinearVelocity);
}

void FourDOFPropulsion::rudderCallback(const std_msgs::Float64::ConstPtr& val)
{
    tf2::Vector3 updatedAngularVelocity = vehicleState.getAngularVelocity();

    if(std::isfinite(val->data))
    {
        LinearPiecewise::Point vel = rudderFunc.getY(val->data);
        if(std::isfinite(vel.y))
        {
            updatedAngularVelocity.setZ(vel.y);
        }
    }

    vehicleState.setAngularVelocity(updatedAngularVelocity);
}