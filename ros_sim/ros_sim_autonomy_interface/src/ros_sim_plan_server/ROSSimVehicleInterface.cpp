#include "ros_sim_plan_server/ROSSimVehicleInterface.h"

#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "std_msgs/String.h"

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

    poseSub = nh.subscribe("primary_navigation", 1, &ROSSimVehicleInterface::navigationFilterCallback, this);
    goalPub = nh.advertise<std_msgs::String>("goal", 1, true);
}

void ROSSimVehicleInterface::sendGoalStatus(GoalStatus status)
{
    std_msgs::String msg;
    if(status == GoalStatus::RUNNING)
    {
         msg.data = "running";
    }
    else if(status == GoalStatus::SUCCESS)
    {
        msg.data = "success";
    }
    else if(status == GoalStatus::FAILED)
    {
        msg.data = "failed";
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

void ROSSimVehicleInterface::registerDataCallback(std::function<void(const PlannerData&)> cb)
{
    dataCallbacks.push_back(cb);
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

VehiclePose ROSSimVehicleInterface::getPosition()
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