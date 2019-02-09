#include <algorithm>
#include <limits>

#include "ros/ros.h"
#include "tf2/LinearMath/Vector3.h"

#include "vehicles/FourDOFPropulsion.h"
#include "model_server/GetModelData.h"

#include "underwater_vehicle_msgs/FloatMeasurement.h"

#define SECONDS_IN_DAY 86400

FourDOFPropulsion::FourDOFPropulsion(VehicleState& vehicleState) :
	PropulsionModule("FourDOFPropulsion", vehicleState),
    forwardThrust(0),
	lateralThrust(0),
	verticalThrust(0),
	rudder(0)
{
    ros::NodeHandle nhPriv("~");
    
    int randomSeed = 0;
    if(nhPriv.getParam("propulsion_random_seed", randomSeed))
    {
         generator = std::default_random_engine(randomSeed);
    }

    nhPriv.param("thrust_sensor_random_noise", thrustSensorRandomNoise, 0.0);
    nhPriv.param("rudder_sensor_random_noise", rudderSensorRandomNoise, 0.0);

    thrustSensorDistribution = std::normal_distribution<double>(0, sqrt(thrustSensorRandomNoise));
	rudderSensorDistribution = std::normal_distribution<double>(0, sqrt(rudderSensorRandomNoise));

    if(nhPriv.hasParam("forward_thruster_thrust") && 
       nhPriv.hasParam("forward_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nhPriv.getParam("forward_thruster_thrust", thrust);
        nhPriv.getParam("forward_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {

            if(thrust.size() == 0)
            {
                ROS_WARN("%s/forward_thruster_thrust and %s/forward_thruster_velocity are empty.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
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
            ROS_WARN("%s/forward_thruster_thrust and %s/forward_thruster_velocity have differing length.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/forward_thruster_thrust or %s/forward_thruster_velocity does not exist.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
    }

    if(nhPriv.hasParam("lateral_thruster_thrust") && 
       nhPriv.hasParam("lateral_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nhPriv.getParam("lateral_thruster_thrust", thrust);
        nhPriv.getParam("lateral_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {
            if(thrust.size() == 0)
            {
                ROS_WARN("%s/lateral_thruster_thrust and %s/lateral_thruster_velocity are empty.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
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
            ROS_WARN("%s/lateral_thruster_thrust and %s/lateral_thruster_velocity have differing length.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/lateral_thruster_thrust or %s/lateral_thruster_velocity does not exist.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
    }

    if(nhPriv.hasParam("vertical_thruster_thrust") && 
       nhPriv.hasParam("vertical_thruster_velocity"))
    {
        std::vector<double> thrust;
        std::vector<double> velocity;
        nhPriv.getParam("vertical_thruster_thrust", thrust);
        nhPriv.getParam("vertical_thruster_velocity", velocity);

        if(thrust.size() == velocity.size())
        {
            if(thrust.size() == 0)
            {
                ROS_WARN("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity are empty.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
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
            ROS_WARN("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity have differing length.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/vertical_thruster_thrust or %s/vertical_thruster_velocity does not exist.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
    }

    if(nhPriv.hasParam("rudder_angle") && 
       nhPriv.hasParam("rudder_velocity"))
    {
        std::vector<double> angle;
        std::vector<double> velocity;
        nhPriv.getParam("rudder_angle", angle);
        nhPriv.getParam("rudder_velocity", velocity);

        if(angle.size() == velocity.size())
        {
            if(angle.size() == 0)
            {
                ROS_WARN("%s/rudder_angle and %s/rudder_velocity are empty.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
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
            ROS_WARN("%s/rudder_angle and %s/rudder_velocity have differing length.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        }
    }
    else
    {
        ROS_WARN("%s/rudder_angle or %s/rudder_velocity does not exist.  Default values will be used.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
    }

    forwardThrusterSub = nh.subscribe("command_forward_thruster", 1, &FourDOFPropulsion::forwardThrusterCallback, this);
	lateralThrusterSub = nh.subscribe("command_lateral_thruster", 1, &FourDOFPropulsion::lateralThrusterCallback, this);
    verticalThrusterSub = nh.subscribe("command_vertical_thruster", 1, &FourDOFPropulsion::verticalThrusterCallback, this);
	rudderSub = nh.subscribe("command_rudder", 1, &FourDOFPropulsion::rudderCallback, this);

    forwardThrusterPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("measured_forward_thruster", 10);
	lateralThrusterPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("measured_lateral_thruster", 10);
	verticalThrusterPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("measured_vertical_thruster", 10);
	rudderPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("measured_rudder", 10);

    vehicleState.setLinearVelocityNED(tf2::Vector3(0,0,0));
    vehicleState.setAngularVelocityNED(tf2::Vector3(0,0,0));
}

void FourDOFPropulsion::forwardThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    forwardThrust = val->data;
}

void FourDOFPropulsion::lateralThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    lateralThrust = val->data;
}

void FourDOFPropulsion::verticalThrusterCallback(const std_msgs::Float64::ConstPtr& val)
{
    verticalThrust = val->data;
}

void FourDOFPropulsion::rudderCallback(const std_msgs::Float64::ConstPtr& val)
{
    rudder = val->data;
}

void FourDOFPropulsion::updateTwist()
{
    tf2::Vector3 updatedLinearVelocity = vehicleState.getLinearVelocityNED();
    tf2::Vector3 updatedAngularVelocity = vehicleState.getAngularVelocityNED();

    if(std::isfinite(forwardThrust))
    {
        LinearPiecewise::Point vel = forwardThrusterFunc.getY(forwardThrust);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setX(vel.y);
        }
    }

    if(std::isfinite(lateralThrust))
    {
        LinearPiecewise::Point vel = lateralThrusterFunc.getY(lateralThrust);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setY(vel.y);
        }
    }

    if(std::isfinite(verticalThrust))
    {
        LinearPiecewise::Point vel = verticalThrusterFunc.getY(verticalThrust);
        if(std::isfinite(vel.y))
        {
            updatedLinearVelocity.setZ(vel.y);
        }
    }

    if(std::isfinite(rudder))
    {
        LinearPiecewise::Point vel = rudderFunc.getY(rudder);
        if(std::isfinite(vel.y))
        {
            updatedAngularVelocity.setZ(vel.y);
        }
    }

    vehicleState.setLinearVelocityNED(updatedLinearVelocity);
    vehicleState.setAngularVelocityNED(updatedAngularVelocity);
}

void FourDOFPropulsion::publishSensors()
{   
    double forwardError = thrustSensorDistribution(generator);
    double lateralError = thrustSensorDistribution(generator);
    double verticalError = thrustSensorDistribution(generator);
    double rudderError = rudderSensorDistribution(generator);

    double forwardMeasurment = forwardThrust + forwardError;
    if((forwardThrust > 0 && forwardMeasurment < 0) ||
       (forwardThrust < 0 && forwardMeasurment > 0))
    {
        forwardMeasurment = 0;
    }

    double lateralMeasurment = lateralThrust + lateralError;
    if((lateralThrust > 0 && lateralMeasurment < 0) ||
       (lateralThrust < 0 && lateralMeasurment > 0))
    {
        lateralMeasurment = 0;
    }

    double verticalMeasurment = verticalThrust + verticalError;
    if((verticalThrust > 0 && verticalMeasurment < 0) ||
       (verticalThrust < 0 && verticalMeasurment > 0))
    {
        verticalMeasurment = 0;
    }

    double rudderMeasurment = rudder + rudderError;
    if((rudder > 0 && rudderMeasurment < 0) ||
       (rudder < 0 && rudderMeasurment > 0))
    {
        rudderMeasurment = 0;
    }

    ros::Time currentTime = ros::Time::now();

    underwater_vehicle_msgs::FloatMeasurementPtr forwardMsg(new underwater_vehicle_msgs::FloatMeasurement);
	forwardMsg->header.frame_id = "world_ned";
	forwardMsg->header.stamp = currentTime;
	forwardMsg->data = forwardMeasurment;
	forwardMsg->variance = thrustSensorRandomNoise;

    underwater_vehicle_msgs::FloatMeasurementPtr lateralMsg(new underwater_vehicle_msgs::FloatMeasurement);
	lateralMsg->header.frame_id = "world_ned";
	lateralMsg->header.stamp = currentTime;
	lateralMsg->data = lateralMeasurment;
	lateralMsg->variance = thrustSensorRandomNoise;

    underwater_vehicle_msgs::FloatMeasurementPtr verticalMsg(new underwater_vehicle_msgs::FloatMeasurement);
	verticalMsg->header.frame_id = "world_ned";
	verticalMsg->header.stamp = currentTime;
	verticalMsg->data = verticalMeasurment;
	verticalMsg->variance = thrustSensorRandomNoise;

    underwater_vehicle_msgs::FloatMeasurementPtr rudderMsg(new underwater_vehicle_msgs::FloatMeasurement);
	rudderMsg->header.frame_id = "world_ned";
	rudderMsg->header.stamp = currentTime;
	rudderMsg->data = rudderMeasurment;
	rudderMsg->variance = thrustSensorRandomNoise;

    forwardThrusterPub.publish(forwardMsg);
	lateralThrusterPub.publish(lateralMsg);
	verticalThrusterPub.publish(verticalMsg);
	rudderPub.publish(rudderMsg);
}

void FourDOFPropulsion::update()
{
    updateTwist();
    publishSensors();
}
