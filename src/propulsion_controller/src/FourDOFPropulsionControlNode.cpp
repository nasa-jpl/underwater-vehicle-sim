#include "ros/ros.h"

#include "propulsion_controller/PropulsionController.h"

#include "propulsion_controller/FourDOFPropulsionLogic.h"
#include "propulsion_controller/FourDOFPropulsionPIDLogic.h"

#include "ros_underwater_sim_utilities/LinearPiecewise.h"

#include "underwater_vehicle_msgs/FloatMeasurement.h"

LinearPiecewise forwardThruster;
LinearPiecewise verticalThruster;

ros::Publisher forwardVelocityPub;
ros::Publisher verticalVelocityPub;

double thrusterStdDev;

LinearPiecewise loadForwardThruster() {
    ros::NodeHandle nhPriv("~");

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
                ROS_FATAL("%s/forward_thruster_thrust and %s/forward_thruster_velocity are empty.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
                exit(1);
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < thrust.size(); i++)
                {
                    points.push_back({velocity[i], thrust[i]});
                }
                return LinearPiecewise(points);
            }
        }
        else
        {
            ROS_FATAL("%s/forward_thruster_thrust and %s/forward_thruster_velocity have differing length.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
            exit(1);
        }
    }
    else
    {
        ROS_FATAL("%s/forward_thruster_thrust or %s/forward_thruster_velocity does not exist.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        exit(1);
    }

    return LinearPiecewise();
}

LinearPiecewise loadVerticalThruster() {
    ros::NodeHandle nhPriv("~");

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
                ROS_FATAL("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity are empty.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
                exit(1);
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < thrust.size(); i++)
                {
                    points.push_back({velocity[i], thrust[i]});
                }
                return LinearPiecewise(points);
            }
        }
        else
        {
            ROS_FATAL("%s/vertical_thruster_thrust and %s/vertical_thruster_velocity have differing length.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
            exit(1);
        }
    }
    else
    {
        ROS_FATAL("%s/vertical_thruster_thrust or %s/vertical_thruster_velocity does not exist.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        exit(1);
    }

    return LinearPiecewise();
}

LinearPiecewise loadRudder() {
    ros::NodeHandle nhPriv("~");

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
                ROS_FATAL("%s/rudder_angle and %s/rudder_velocity are empty.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
                exit(1);
            }
            else
            {
                std::vector<LinearPiecewise::Point> points;
                for(unsigned int i = 0; i < angle.size(); i++)
                {
                    points.push_back({velocity[i], angle[i]});
                }
                return LinearPiecewise(points);
            }
        }
        else
        {
            ROS_FATAL("%s/rudder_angle and %s/rudder_velocity have differing length.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
            exit(1);
        }
    }
    else
    {
        ROS_FATAL("%s/rudder_angle or %s/rudder_velocity does not exist.", nhPriv.getNamespace().c_str(), nhPriv.getNamespace().c_str());
        exit(1);
    }

    return LinearPiecewise();
}

/**
* Callback that takes in the commanded forward thrust value and converts it into an expected velocity.
* This velocity is then published for use as an expected vehicle velocity.
*/
void forwardThrusterCallback(const std_msgs::Float64::ConstPtr& val) {
    underwater_vehicle_msgs::FloatMeasurement velData;
    
    std::vector<LinearPiecewise::Point> possibleX = forwardThruster.getX(val->data);
    if(possibleX.size() > 0) {
        velData.data = possibleX[0].x;
        velData.variance = thrusterStdDev*thrusterStdDev;
        velData.header.stamp = ros::Time::now();
        forwardVelocityPub.publish(velData);
    }
}

/**
* Callback that takes in the commanded vertical thrust value and converts it into an expected velocity.
* This velocity is then published for use as an expected vehicle velocity.
*/
void verticalThrusterCallback(const std_msgs::Float64::ConstPtr& val) {
    underwater_vehicle_msgs::FloatMeasurement velData;
    
    std::vector<LinearPiecewise::Point> possibleX = verticalThruster.getX(val->data);
    if(possibleX.size() > 0) {
        velData.data = possibleX[0].x;
        velData.variance = thrusterStdDev*thrusterStdDev;
        velData.header.stamp = ros::Time::now();
        verticalVelocityPub.publish(velData);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "four_dof_propulsion_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    std::unique_ptr<PropulsionController> controller;
 
    ros::ServiceClient infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    infoClient.waitForExistence();

    underwater_vehicle_msgs::GetVehicleInfo info;
    infoClient.call(info);
    VehicleInfo vehicleInfo(info);

    ros::Subscriber forwardThrusterSub = nh.subscribe("command_forward_thruster", 10, &forwardThrusterCallback);
    forwardVelocityPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("commanded_forward_velocity", 1000);

    ros::Subscriber verticalThrusterSub = nh.subscribe("command_vertical_thruster", 10, &verticalThrusterCallback);
    verticalVelocityPub = nh.advertise<underwater_vehicle_msgs::FloatMeasurement>("commanded_vertical_velocity", 1000);

    bool usePID;
    if(!nhPriv.getParam("use_pid", usePID))
    {
        ROS_FATAL("Parameter \"use_pid\" not present in the parameter server.");
        exit(1);
    }

    if(!nhPriv.getParam("thruster_velocity_std_dev", thrusterStdDev)) {
        thrusterStdDev = 0.25;
    }

    if(usePID)
    {
        std::unique_ptr<PropulsionLogicInterface> logic(new FourDOFPropulsionPIDLogic(vehicleInfo));
        controller.reset(new PropulsionController(vehicleInfo, std::move(logic)));
    }
    else
    {
        forwardThruster = loadForwardThruster();
        verticalThruster = loadVerticalThruster();
        LinearPiecewise rudder = loadRudder();

        std::unique_ptr<PropulsionLogicInterface> logic(new FourDOFPropulsionLogic(vehicleInfo, 
                                                                                   forwardThruster, 
                                                                                   verticalThruster, 
                                                                                   rudder));
        controller.reset(new PropulsionController(vehicleInfo, std::move(logic)));
    }

    ros::spin();
    return 0;
}