#include "ROSSimVehicleInterface.h"

#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/sensor_data_types/CommonDataTypes.h"

#include "std_msgs/String.h"

using namespace underwater_autonomy;

ROSSimVehicleInterface::ROSSimVehicleInterface(VehicleInfo info) :
    info(info),
    listener(buffer)
{
    ros::NodeHandle nh;
    std::vector<std::string> data = info.getModuleNamesOfType("DataBroadcaster");
    if(data.size() > 0)
    {
        dataSub = nh.subscribe(data[0] + "/data", 1, &ROSSimVehicleInterface::receiveData, this);
    }
    else
    {
        ROS_FATAL("No DataBroadcaster module in vehicle");
    }

    std::vector<std::string> usblData = info.getModuleNamesOfType("USBL");
    if(usblData.size() > 0)
    {
        usblSub = nh.subscribe(usblData[0] + "/data", 1, &ROSSimVehicleInterface::receiveUSBL, this);
    }

    poseSub = nh.subscribe("primary_navigation", 1, &ROSSimVehicleInterface::navigationFilterCallback, this);
    goalPub = nh.advertise<std_msgs::String>("goal", 1, true);

    tfTimer = nh.createTimer(ros::Duration(0.25), &ROSSimVehicleInterface::receivePose, this);
    lastTFTime = std::numeric_limits<double>::quiet_NaN();
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

void ROSSimVehicleInterface::receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg)
{
    double time = msg->time.toSec();
    VehiclePose pose(Eigen::Vector3d(msg->x, msg->y, msg->h));
    std::map<std::string, double> data;

    data["sonar_depth"] = msg->sonarDepth;
    data["temp"] = msg->temp;
    data["salt"] = msg->salt;
    data["dye"] = msg->dye;
    data["plume"] = msg->dye;

    PlannerData plannerData(time, pose, data);

    for(std::function<void(const PlannerData&)> cb : dataCallbacks)
    {
        cb(plannerData);
    }
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
    }

    if(info.getPropModuleType() == "FourDOFPropulsion")
    {
        ros::NodeHandle nh(info.getPropModuleName());

        forwardThrusterSub = nh.subscribe("measured_forward_thruster", 
                                               100, 
                                               &ROSSimVehicleInterface::receiveForwardThruster, 
                                               this);

        lateralThrusterSub = nh.subscribe("measured_lateral_thruster", 
                                               100, 
                                               &ROSSimVehicleInterface::receiveLateralThruster, 
                                               this);
    }
}

void ROSSimVehicleInterface::receivePose(const ros::TimerEvent& event)
{
    geometry_msgs::TransformStamped transformMsg;
    ros::NodeHandle nh;
    std::string vehicleName = nh.getNamespace().substr(1);
	try
    {
        if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(1.0)))
        {
            transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));

            VehiclePose pose;
            Eigen::Vector3d position (transformMsg.transform.translation.x,
                                      transformMsg.transform.translation.y,
                                      transformMsg.transform.translation.z);
            Eigen::Quaterniond orientation(transformMsg.transform.rotation.w,
                                           transformMsg.transform.rotation.x,
                                           transformMsg.transform.rotation.y,
                                           transformMsg.transform.rotation.z);

            if(std::isnan(lastTFTime)) {
                lastTfPose = pose;
                lastTFTime = transformMsg.header.stamp.toSec();
            }

            double timeDelta = transformMsg.header.stamp.toSec() - lastTFTime;
            //Extract new lateral velocity
            Eigen::Vector3d linearVelocity = (position - lastTfPose.getPosition()) / timeDelta;
            linearVelocity = orientation.inverse() * linearVelocity; //Rotate linear velocity into body frame from world frame
            
            //Extract new rotational velocity
            Eigen::Quaterniond rotation = orientation  * lastTfPose.getOrientation().inverse();

            Eigen::AngleAxisd rotationAA(rotation);
            Eigen::Vector3d angularVelocity = rotationAA.axis() * (rotationAA.angle() / timeDelta);
            angularVelocity = orientation.inverse() * angularVelocity; //Rotate angular velocity into body frame from world frame
            
            
            pose.setPosition(position);
            pose.setOrientation(orientation);    
            pose.setLinearVelocity(linearVelocity);
            pose.setAngularVelocity(angularVelocity);

            publishDataToCallbacks<VehiclePose>("true_pose", pose);

            lastTfPose = pose;
            lastTFTime = transformMsg.header.stamp.toSec();
        }
	}
	catch(tf2::TransformException ex)
	{
		throw std::move(ex);
	}   
}

void ROSSimVehicleInterface::receiveIMU(sensor_msgs::Imu msgData)
{
    DoubleSensorData heading;
    Vector3DSensorData angularVelocity;

    tf2::Quaternion orientation(msgData.orientation.x,
                                msgData.orientation.y,
                                msgData.orientation.z,
                                msgData.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    heading.data = yaw;
    heading.time = msgData.header.stamp.toSec();

    angularVelocity.x = msgData.angular_velocity.x;
    angularVelocity.y = msgData.angular_velocity.y;
    angularVelocity.z = msgData.angular_velocity.z;
    angularVelocity.time = msgData.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("heading", heading);
    publishDataToCallbacks<Vector3DSensorData>("angular_velocity", angularVelocity);
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

void ROSSimVehicleInterface::receiveForwardThruster(underwater_vehicle_msgs::FloatMeasurement forwardData)
{
    DoubleSensorData forward;

    forward.data = forwardData.data;
    forward.time = forwardData.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("forwater_thruster", forward);
}

void ROSSimVehicleInterface::receiveLateralThruster(underwater_vehicle_msgs::FloatMeasurement lateralData)
{
    DoubleSensorData lateral;

    lateral.data = lateralData.data;
    lateral.time = lateralData.header.stamp.toSec();

    publishDataToCallbacks<DoubleSensorData>("lateral_thruster", lateral);
} 