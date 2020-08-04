#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/sensor_data_types/CommonDataTypes.h"

#include "std_msgs/String.h"

using namespace underwater_autonomy;

ROSSimVehicleInterface::ROSSimVehicleInterface(VehicleInfo info) :
    info(info)
{
    ros::NodeHandle nh;

    poseSub = nh.subscribe("primary_navigation", 1, &ROSSimVehicleInterface::navigationFilterCallback, this);
    goalPub = nh.advertise<std_msgs::String>("goal", 1, true);

    initializeCallbacks();
}

void ROSSimVehicleInterface::sendPlannerStatus(PlannerStatus status)
{
    std_msgs::String msg;
    if(status == PlannerStatus::RUNNING)
    {
        log(LogLevel::INFO, "Planner Status: Running");
        msg.data = "running";
    }
    else if(status == PlannerStatus::SUCCESS)
    {
        log(LogLevel::INFO, "Planner Status: Success");
        msg.data = "success";
    }
    else if(status == PlannerStatus::FAILED)
    {
        log(LogLevel::INFO, "Planner Status: Failed");
        msg.data = "failed";
    }
    else if(status == PlannerStatus::PAUSED)
    {
        log(LogLevel::INFO, "Planner Status: Paused");
        msg.data = "paused";
    }
    else
    {
        log(LogLevel::WARN, "Unexpected Planner Status");
        msg.data = "unexpected_status";
    }

    goalPub.publish(msg);
}

void ROSSimVehicleInterface::log(LogLevel level, std::string string)
{
    if(level == LogLevel::DEBUG)
    {
        ROS_DEBUG("%s", (info.getName() + ": " + string).c_str());
    }
    else if(level == LogLevel::INFO)
    {
        ROS_INFO("%s", (info.getName() + ": " + string).c_str());
    }
    else if(level == LogLevel::WARN)
    {
        ROS_WARN("%s", (info.getName() + ": " + string).c_str());
    }
    else if(level == LogLevel::ERROR)
    {
        ROS_ERROR("%s", (info.getName() + ": " + string).c_str());
    }
    else if(level == LogLevel::FATAL)
    {
        ROS_FATAL("%s", (info.getName() + ": " + string).c_str());
    }
}

double ROSSimVehicleInterface::getTime() const
{
    return ros::Time::now().toSec();
}

VehiclePose ROSSimVehicleInterface::getPosition() const
{
    return currentPose;
}

void ROSSimVehicleInterface::navigationFilterCallback(const nav_msgs::Odometry odo)
{
	Eigen::Vector3d position(odo.pose.pose.position.x,
							 odo.pose.pose.position.y,
							 odo.pose.pose.position.z);

	Eigen::Quaterniond orientation(odo.pose.pose.orientation.w,
								   odo.pose.pose.orientation.x,
								   odo.pose.pose.orientation.y,
								   odo.pose.pose.orientation.z);

	Eigen::Matrix<double,6,6> poseCovariance;
	for(unsigned int i = 0; i < 6; i++)
	{
		for(unsigned int j = 0; j < 6; j++)
		{
			poseCovariance(i, j) = odo.pose.covariance[(i * 6) + j];
		}
	}


	Eigen::Vector3d linearVelocity(odo.twist.twist.linear.x,
								   odo.twist.twist.linear.y,
								   odo.twist.twist.linear.z);
	Eigen::Vector3d angularVelocity(odo.twist.twist.angular.x,
								    odo.twist.twist.angular.y,
									odo.twist.twist.angular.z);

	Eigen::Matrix<double,6,6> twistCovariance;
	for(unsigned int i = 0; i < 6; i++)
	{
		for(unsigned int j = 0; j < 6; j++)
		{
			twistCovariance(i, j) = odo.twist.covariance[(i * 6) + j];
		}
	}

	currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);

    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);
}

VehicleInfo ROSSimVehicleInterface::getVehicleInfo() {
    return info;
}

void ROSSimVehicleInterface::initializeCallbacks() {
    std::vector<std::string> moduleNames = info.getModuleNames();
    std::vector<std::string> moduleTypes = info.getModuleTypes();

    for(unsigned int i = 0; i < moduleNames.size(); i++)
    {
        ros::NodeHandle nh(moduleNames[i]);
        if(moduleTypes[i] == "IMU")
        {
            imuSub = nh.subscribe("data", 
                                       100, 
                                       &ROSSimVehicleInterface::receiveIMU, 
                                       this);

        }
        else if(moduleTypes[i] == "USBL")
        {
            usblSub = nh.subscribe("data", 
                                        100, 
                                        &ROSSimVehicleInterface::receiveUSBL, 
                                        this);
        }
        else if(moduleTypes[i] == "Depth")
        {
            depthSub = nh.subscribe("data", 
                                     100, 
                                     &ROSSimVehicleInterface::receiveDepth, 
                                     this);
        }
        else if(moduleTypes[i] == "DVL")
        {
            dvlSub = nh.subscribe("data",
                                     100, 
                                     &ROSSimVehicleInterface::receiveDVL, 
                                     this);
        } 
        else if (moduleTypes[i] == "DataBroadcaster") 
        {
            dataSub = nh.subscribe("data", 
                                   100, 
                                   &ROSSimVehicleInterface::receiveModelData, 
                                   this);
        }
    }
    ros::NodeHandle nh;
    forwardVelSub = nh.subscribe("commanded_forward_velocity", 
                                        100, 
                                        &ROSSimVehicleInterface::receiveCommandedFowardVelocity, 
                                        this);
    verticalVelSub = nh.subscribe("commanded_vertical_velocity", 
                                        100, 
                                        &ROSSimVehicleInterface::receiveCommandedVerticalVelocity, 
                                        this);

}

void ROSSimVehicleInterface::receiveModelData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg)
{
    DoubleSensorData sonarDepth;
    DoubleSensorData temp;
    DoubleSensorData salt;
    DoubleSensorData dye;

    sonarDepth.data = msg->sonarDepth;
    sonarDepth.time = msg->time.toSec();

    temp.data = msg->temp;
    temp.time = msg->time.toSec();

    salt.data = msg->salt;
    salt.time = msg->time.toSec();

    dye.data = msg->dye;
    dye.time = msg->time.toSec();

    publishDataToCallbacks<DoubleSensorData>("sonar_depth", sonarDepth);
    publishDataToCallbacks<DoubleSensorData>("temp", temp);
    publishDataToCallbacks<DoubleSensorData>("salt", salt);
    publishDataToCallbacks<DoubleSensorData>("dye", dye);
    publishDataToCallbacks<DoubleSensorData>("plume", dye);
}

void ROSSimVehicleInterface::receiveIMU(sensor_msgs::Imu msgData)
{
    DoubleSensorData heading;
    Vector3dData angularVelocity;

    if(msgData.orientation_covariance[0] != -1 &&
       !std::isnan(msgData.orientation.x) &&
       !std::isnan(msgData.orientation.y) &&
       !std::isnan(msgData.orientation.z) &&
       !std::isnan(msgData.orientation.w))
    {
        tf2::Quaternion orientation(msgData.orientation.x,
                                        msgData.orientation.y,
                                        msgData.orientation.z,
                                        msgData.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            heading.data = yaw;
            heading.time = msgData.header.stamp.toSec();
            publishDataToCallbacks<DoubleSensorData>("heading", heading);
    } 
    
    if(msgData.angular_velocity_covariance[0] != -1 &&
       !std::isnan(msgData.angular_velocity.x) &&
       !std::isnan(msgData.angular_velocity.y) &&
       !std::isnan(msgData.angular_velocity.z))
    {
        angularVelocity.data[0] = msgData.angular_velocity.x;
        angularVelocity.data[1] = msgData.angular_velocity.y;
        angularVelocity.data[2] = msgData.angular_velocity.z;
        angularVelocity.time = msgData.header.stamp.toSec();

        publishDataToCallbacks<Vector3dData>("angular_velocity", angularVelocity);
    }
}

void ROSSimVehicleInterface::receiveUSBL(underwater_vehicle_msgs::USBL msgData)
{
    USBLSensorData usbl;

    usbl.range = msgData.range;
    usbl.bearing = msgData.bearing;
    usbl.time = msgData.header.stamp.toSec();

    publishDataToCallbacks<USBLSensorData>("usbl", usbl);
}

void ROSSimVehicleInterface::receiveDVL(underwater_vehicle_msgs::DVL msgData)
{
    DVLSensorData dvl;

    dvl.x = msgData.velocity.x;
    dvl.y = msgData.velocity.y;
    dvl.z = msgData.velocity.z;
    dvl.time = msgData.header.stamp.toSec();

    if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_WATER)
    {
        dvl.velocityReference = DVLSensorData::VelocityReference::WATER;
    }
    else if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_BOTTOM)
    {
        dvl.velocityReference = DVLSensorData::VelocityReference::BOTTOM;
    } else {
        dvl.velocityReference = DVLSensorData::VelocityReference::UNKNOWN;
    }

    publishDataToCallbacks<DVLSensorData>("dvl", dvl);
}

void ROSSimVehicleInterface::receiveDepth(underwater_vehicle_msgs::FloatMeasurement msgData)
{
    DoubleSensorData depth;

    depth.data = msgData.data;
    depth.time = msgData.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("depth", depth);
}

void ROSSimVehicleInterface::receiveCommandedFowardVelocity(underwater_vehicle_msgs::FloatMeasurement commandedForwardVelocity) {
    DoubleSensorData forwardData;

    forwardData.data = commandedForwardVelocity.data;
    forwardData.time = commandedForwardVelocity.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("commanded_forward_velocity", forwardData);
}

void ROSSimVehicleInterface::receiveCommandedVerticalVelocity(underwater_vehicle_msgs::FloatMeasurement commandedVerticalVelocity) {
    DoubleSensorData verticalData;

    verticalData.data = commandedVerticalVelocity.data;
    verticalData.time = commandedVerticalVelocity.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("commanded_vertical_velocity", verticalData);
}