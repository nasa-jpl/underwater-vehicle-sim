#include "ros_sim_navigation/ROSSimNavigationFilter.h"

#include "underwater_autonomy/navigation/TrueNavigationFilter.h"
#include "underwater_autonomy/navigation/DeadReckoningNavigationFilter.h"

#include "nav_msgs/Odometry.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleInfo.h"

using namespace underwater_autonomy;

ROSSimNavigationFilter::ROSSimNavigationFilter(std::string filterName, VehicleInfo info) :
    info(info),
    filterName(filterName),
    listener(buffer)
{
    initializeNavFilter(filterName, info);

    ros::NodeHandle filterNh("nav_filters");
    posePublisher = filterNh.advertise<nav_msgs::Odometry>(filterName, 1);

    initializeCallbacks(filterName, info);    
}

ROSSimNavigationFilter::ROSSimNavigationFilter(ROSSimNavigationFilter&& other) :
    info(std::move(other.info)),
    filterName(std::move(other.filterName)),
    listener(buffer),
    posePublisher(std::move(other.posePublisher)),
    filter(std::move(other.filter)),
    imuData(std::move(other.imuData)),
    usblData(std::move(other.usblData)),
    depthData(std::move(other.depthData)),
    forwardThrusterData(std::move(other.forwardThrusterData)),
    lateralThrusterData(std::move(other.lateralThrusterData))
{
    initializeCallbacks(filterName, info);
}

void ROSSimNavigationFilter::initializeCallbacks(std::string& filterName, VehicleInfo& info)
{
    std::vector<std::string> moduleNames = info.getModuleNames();
    std::vector<std::string> moduleTypes = info.getModuleTypes();

    for(unsigned int i = 0; i < moduleNames.size(); i++)
    {
        ros::NodeHandle nh(moduleNames[i]);

        if(moduleTypes[i] == "IMU")
        {
            imuData = nh.subscribe("data", 
                                       100, 
                                       &ROSSimNavigationFilter::sendIMUToFilter, 
                                       this);
        }
        else if(moduleTypes[i] == "USBL")
        {
            usblData = nh.subscribe("data", 
                                        100, 
                                        &ROSSimNavigationFilter::sendUSBLToFilter, 
                                        this);
        }
        else if(moduleTypes[i] == "Depth")
        {
            depthData = nh.subscribe("data", 
                                     100, 
                                     &ROSSimNavigationFilter::sendDepthToFilter, 
                                     this);
        }
        else if(moduleTypes[i] == "DVL")
        {
            dvlData = nh.subscribe("data",
                                     100, 
                                     &ROSSimNavigationFilter::sendDVLToFilter, 
                                     this);
        }
    }

    if(info.getPropModuleType() == "FourDOFPropulsion")
    {
        ros::NodeHandle nh(info.getPropModuleName());

        forwardThrusterData = nh.subscribe("measured_forward_thruster", 
                                               100, 
                                               &ROSSimNavigationFilter::sendForwardThruster, 
                                               this);

        lateralThrusterData = nh.subscribe("measured_lateral_thruster", 
                                               100, 
                                               &ROSSimNavigationFilter::sendLateralThruster, 
                                               this);
        //subscribe to thruster info
    }
}

void ROSSimNavigationFilter::initializeNavFilter(std::string& filterName, VehicleInfo& info)
{
    ros::NodeHandle filterNhPriv("~/filter/" + filterName);
    ros::NodeHandle filterNh("nav_filters");

    std::string filterType;
    if(!filterNhPriv.getParam("type", filterType))
    {
        ROS_INFO("Parameter \"%s/type\" not present in the parameter server.", filterNhPriv.getNamespace().c_str());
        exit(1);
    }


    if(filterType == "TrueNavigation")
    {
        VehiclePose startPose(Eigen::Vector3d(info.getStartX(), info.getStartY(), info.getStartZ()));
        filter.reset(new TrueNavigationFilter(startPose, ros::Time::now().toSec()));
    }
    else if(filterType == "DeadReckoning")
    {
        VehiclePose startPose(Eigen::Vector3d(info.getStartX(), info.getStartY(), info.getStartZ()));
        filter.reset(new DeadReckoningNavigationFilter(startPose, ros::Time::now().toSec()));
    }

}

std::vector<std::vector<double>> ROSSimNavigationFilter::get2dArrayParam(ros::NodeHandle nh, std::string name, std::vector<std::vector<double>> defaultVal)
{
    std::vector<std::vector<double>> returnList;

    XmlRpc::XmlRpcValue list;
    if(nh.getParam(name, list))
    {
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for(int i = 0; i < list.size(); i++)
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            returnList.push_back({});
            for(int j = 0; j < list[i].size(); j++)
            {
                ROS_ASSERT(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                           list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
                if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    returnList[i].push_back(static_cast<double>(list[i][j]));
                }
                else if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    returnList[i].push_back(static_cast<int>(list[i][j]));
                }
            }
        }
    }
    else
    {
        returnList = defaultVal;
    
    }
    return returnList;
}

void ROSSimNavigationFilter::update()
{
    std::vector<double> inputs;
    double currentTime = ros::Time::now().toSec();
    filter->predict(currentTime, inputs);
    sendPoseToFilter();
}

void ROSSimNavigationFilter::publishPose()
{
    VehiclePose pose = filter->getPoseEstimation();

    Eigen::Vector3d position = pose.getPosition();
    Eigen::Quaterniond orientation = pose.getOrientation();
    Eigen::Matrix<double,6,6> poseCovariance = pose.getPoseCovariance();

    Eigen::Vector3d linearVelocity = pose.getLinearVelocity();
    Eigen::Vector3d angularVelocity = pose.getAngularVelocity();
    Eigen::Matrix<double,6,6> twistCovariance = pose.getTwistCovariance();

    nav_msgs::Odometry odoMsg;
    odoMsg.header.stamp = ros::Time::now();
    odoMsg.header.frame_id = "world_ned"; //pose frame
    odoMsg.child_frame_id = "world_ned"; //twist frame
    odoMsg.pose.pose.position.x = position[0];
    odoMsg.pose.pose.position.y = position[1];
    odoMsg.pose.pose.position.z = position[2];

    odoMsg.pose.pose.orientation.x = orientation.x();
    odoMsg.pose.pose.orientation.y = orientation.y();
    odoMsg.pose.pose.orientation.z = orientation.z();
    odoMsg.pose.pose.orientation.w = orientation.w();

    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            odoMsg.pose.covariance[(i * 6) + j] = poseCovariance(i, j);
        }
    }

    odoMsg.twist.twist.linear.x = linearVelocity[0];
    odoMsg.twist.twist.linear.y = linearVelocity[1];
    odoMsg.twist.twist.linear.z = linearVelocity[2];

    odoMsg.twist.twist.angular.x = angularVelocity[0];
    odoMsg.twist.twist.angular.y = angularVelocity[1];
    odoMsg.twist.twist.angular.z = angularVelocity[2];

    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            odoMsg.twist.covariance[(i * 6) + j] = twistCovariance(i, j);
        }
    }

    posePublisher.publish(odoMsg);
}

void ROSSimNavigationFilter::sendPoseToFilter()
{
    geometry_msgs::TransformStamped transformMsg;
    ros::NodeHandle nh;
    std::string vehicleName = nh.getNamespace().substr(1);
	try
    {
        if(buffer.canTransform("world_ned", vehicleName, ros::Time(0), ros::Duration(1.0)))
        {
            transformMsg = buffer.lookupTransform("world_ned", vehicleName, ros::Time(0));
            std::vector<double> data;
            std::vector<double> input;
            data.push_back(transformMsg.transform.translation.x); //x position
            data.push_back(transformMsg.transform.translation.y); //y position
            data.push_back(transformMsg.transform.translation.z); //z position
            data.push_back(transformMsg.transform.rotation.x); //x orientation
            data.push_back(transformMsg.transform.rotation.y); //y orientation
            data.push_back(transformMsg.transform.rotation.z); //z orientation
            data.push_back(transformMsg.transform.rotation.w); //w orientation

            filter->sensorMeasurement("pose", transformMsg.header.stamp.toSec(), data, input);
        }
	}
	catch(tf2::TransformException ex)
	{
		throw std::move(ex);
	}   
}

void ROSSimNavigationFilter::sendIMUToFilter(sensor_msgs::Imu msgData)
{
    std::vector<double> filterHeadingData;
    std::vector<double> filterVelData;
    std::vector<double> input;

    tf2::Quaternion orientation(msgData.orientation.x,
                                msgData.orientation.y,
                                msgData.orientation.z,
                                msgData.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    filterHeadingData.push_back(yaw);

    filterVelData.push_back(msgData.angular_velocity.x);
    filterVelData.push_back(msgData.angular_velocity.y);
    filterVelData.push_back(msgData.angular_velocity.z);

    filter->sensorMeasurement("heading", msgData.header.stamp.toSec(), filterHeadingData, input);
    filter->sensorMeasurement("angular_velocity", msgData.header.stamp.toSec(), filterVelData, input);
}

void ROSSimNavigationFilter::sendUSBLToFilter(underwater_vehicle_msgs::USBL msgData)
{
    std::vector<double> filterRangeData;
    std::vector<double> filterUSBLData;
    std::vector<double> input;

    input.push_back(msgData.beacon_x);
    input.push_back(msgData.beacon_y);
    input.push_back(msgData.beacon_z);

    filterRangeData.push_back(msgData.range);

    filterUSBLData.push_back(msgData.range);
    filterUSBLData.push_back(msgData.bearing);

    filter->sensorMeasurement("slant_range", msgData.header.stamp.toSec(), filterRangeData, input);
    filter->sensorMeasurement("usbl", msgData.header.stamp.toSec(), filterUSBLData, input);
}

void ROSSimNavigationFilter::sendDVLToFilter(underwater_vehicle_msgs::DVL msgData)
{
    std::vector<double> filterData;
    std::vector<double> input;

    filterData.push_back(msgData.velocity.x);
    filterData.push_back(msgData.velocity.y);
    filterData.push_back(msgData.velocity.z);

    if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_WATER)
    {
        filter->sensorMeasurement("dvl_wrt_water", msgData.header.stamp.toSec(), filterData, input);
    }
    else if(msgData.velocity_reference == msgData.VELOCITY_REFERENCE_BOTTOM)
    {
        filter->sensorMeasurement("dvl_wrt_bottom", msgData.header.stamp.toSec(), filterData, input);
    }
}

void ROSSimNavigationFilter::sendDepthToFilter(underwater_vehicle_msgs::FloatMeasurement msgData)
{
    std::vector<double> filterDepthData;
    std::vector<double> input;

    filterDepthData.push_back(msgData.data);

    filter->sensorMeasurement("depth", msgData.header.stamp.toSec(), filterDepthData, input);
}

void ROSSimNavigationFilter::sendForwardThruster(underwater_vehicle_msgs::FloatMeasurement forwardData)
{
    std::vector<double> filterForwardThrusterData;
    std::vector<double> input;

    filterForwardThrusterData.push_back(forwardData.data);
    
    
    filter->sensorMeasurement("forward_thruster_command", forwardData.header.stamp.toSec(), filterForwardThrusterData, input);
}

void ROSSimNavigationFilter::sendLateralThruster(underwater_vehicle_msgs::FloatMeasurement lateralData)
{
    std::vector<double> filterLateralThrusterData;
    std::vector<double> input;

    filterLateralThrusterData.push_back(lateralData.data);

    filter->sensorMeasurement("lateral_thruster_command", lateralData.header.stamp.toSec(), filterLateralThrusterData, input);
} 