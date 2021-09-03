#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/sensor_data_types/CommonDataTypes.h"

#include "std_msgs/String.h"

#include "underwater_vehicle_msgs/LogData.h"

using namespace underwater_autonomy;

ROSSimVehicleInterface::ROSSimVehicleInterface(VehicleInfo info) :
    info(info)
{
    poseSub = nh.subscribe("primary_navigation", 1, &ROSSimVehicleInterface::navigationFilterCallback, this);
    statusPub = nh.advertise<std_msgs::String>("planner_status", 1, true);

    initializeCallbacks();
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

void ROSSimVehicleInterface::log(std::string channel, underwater_autonomy::LogData data) {

    underwater_vehicle_msgs::LogData logMsg;
    logMsg.header.stamp = ros::Time::now();
    std::vector<underwater_vehicle_msgs::DoubleArray> doubleArrays;
    std::vector<underwater_vehicle_msgs::IntArray> intArrays;
    std::vector<underwater_vehicle_msgs::ByteArray> byteArrays;
    std::vector<underwater_vehicle_msgs::StringArray> stringArrays;

    for(uint i = 0; i < data.numDoubleArrays(); i++) {
        underwater_vehicle_msgs::DoubleArray array;
        array.variable = data.getDoubleVariable(i);
        array.units = data.getDoubleUnits(i);
        array.data = data.getDoubleArray(i);
        doubleArrays.push_back(array);
    }

    for(uint i = 0; i < data.numIntArrays(); i++) {
        underwater_vehicle_msgs::IntArray array;
        array.variable = data.getIntVariable(i);
        array.units = data.getIntUnits(i);
        array.data = data.getIntArray(i);
        intArrays.push_back(array);
    }

    for(uint i = 0; i < data.numByteArrays(); i++) {
        underwater_vehicle_msgs::ByteArray array;
        array.variable = data.getByteVariable(i);
        array.units = data.getByteUnits(i);
        array.data = data.getByteArray(i);
        byteArrays.push_back(array);
    }

    for(uint i = 0; i < data.numStringArrays(); i++) {
        underwater_vehicle_msgs::StringArray array;
        array.variable = data.getStringVariable(i);
        array.units = data.getStringUnits(i);
        array.data = data.getStringArray(i);
        stringArrays.push_back(array);
    }
    logMsg.doubleArrays = doubleArrays;
    logMsg.intArrays = intArrays;
    logMsg.byteArrays = byteArrays;
    logMsg.stringArrays = stringArrays;

    auto entry = logPublishers.find(channel);
    if( entry == logPublishers.end()) {
        ros::Publisher pub = nh.advertise<underwater_vehicle_msgs::LogData>(channel, 10);
        logPublishers.insert(std::pair<std::string,ros::Publisher>(channel, pub));

        //Publish an empty message first so rosbag can then subscribe to the next channel.
        underwater_vehicle_msgs::LogData emptyMsg;
        emptyMsg.header.stamp = ros::Time::now();
        logPublishers[channel].publish(emptyMsg);
    }

    logPublishers[channel].publish(logMsg);
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
            heading.variance = msgData.orientation_covariance[8];
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
        angularVelocity.covariance(0,0) = msgData.angular_velocity_covariance[0];
        angularVelocity.covariance(0,1) = msgData.angular_velocity_covariance[1];
        angularVelocity.covariance(0,2) = msgData.angular_velocity_covariance[2];
        angularVelocity.covariance(1,0) = msgData.angular_velocity_covariance[3];
        angularVelocity.covariance(1,1) = msgData.angular_velocity_covariance[4];
        angularVelocity.covariance(1,2) = msgData.angular_velocity_covariance[5];
        angularVelocity.covariance(2,0) = msgData.angular_velocity_covariance[6];
        angularVelocity.covariance(2,1) = msgData.angular_velocity_covariance[7];
        angularVelocity.covariance(2,2) = msgData.angular_velocity_covariance[8];

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
    usbl.covariance(0,0) = msgData.range_bearing_covariance[0];
    usbl.covariance(0,1) = msgData.range_bearing_covariance[1];
    usbl.covariance(1,0) = msgData.range_bearing_covariance[2];
    usbl.covariance(1,1) = msgData.range_bearing_covariance[3];

    if(std::isnan(usbl.range)) {
        usbl.covariance(0,0) = -1;
    }
    if(std::isnan(usbl.bearing)) {
        usbl.covariance(1,1) = -1;
    }
    
    publishDataToCallbacks<USBLSensorData>("usbl", usbl);
}

void ROSSimVehicleInterface::receiveDVL(underwater_vehicle_msgs::DVL msgData)
{
    Vector3dData dvl;

    dvl.data[0] = msgData.velocity.x;
    dvl.data[1] = msgData.velocity.y;
    dvl.data[2] = msgData.velocity.z;
    dvl.covariance(0,0) = msgData.velocity_covariance[0];
    dvl.covariance(0,1) = msgData.velocity_covariance[1];
    dvl.covariance(0,2) = msgData.velocity_covariance[2];
    dvl.covariance(1,0) = msgData.velocity_covariance[3];
    dvl.covariance(1,1) = msgData.velocity_covariance[4];
    dvl.covariance(1,2) = msgData.velocity_covariance[5];
    dvl.covariance(2,0) = msgData.velocity_covariance[6];
    dvl.covariance(2,1) = msgData.velocity_covariance[7];
    dvl.covariance(2,2) = msgData.velocity_covariance[8];


    dvl.time = msgData.header.stamp.toSec();
    if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_WATER)
    {
            publishDataToCallbacks<Vector3dData>("linear_velocity_wrt_water", dvl);
    }
    else if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_BOTTOM)
    {
            publishDataToCallbacks<Vector3dData>("linear_velocity_wrt_ground", dvl);
    }
}

void ROSSimVehicleInterface::receiveDepth(underwater_vehicle_msgs::FloatMeasurement msgData)
{
    DoubleSensorData depth;

    depth.data = msgData.data;
    depth.time = msgData.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("depth", depth);
}

void ROSSimVehicleInterface::receiveCommandedFowardVelocity(underwater_vehicle_msgs::FloatMeasurement commandedForwardVelocity) {
    Vector3dData forwardData;

    forwardData.data[0] = commandedForwardVelocity.data;
    forwardData.data[1] = 0;
    forwardData.data[2] = 0;

    forwardData.covariance(0,0) = commandedForwardVelocity.variance;
    forwardData.covariance(0,1) = -1;
    forwardData.covariance(0,2) = -1;
    forwardData.covariance(1,0) = -1;
    forwardData.covariance(1,1) = -1;
    forwardData.covariance(1,2) = -1;
    forwardData.covariance(2,0) = -1;
    forwardData.covariance(2,1) = -1;
    forwardData.covariance(2,2) = -1;

    forwardData.time = commandedForwardVelocity.header.stamp.toSec();

    publishDataToCallbacks<Vector3dData>("linear_velocity_wrt_prop", forwardData);
}