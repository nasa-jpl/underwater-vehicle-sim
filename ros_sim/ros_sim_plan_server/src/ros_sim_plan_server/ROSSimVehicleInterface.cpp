#include "ros_sim_plan_server/ROSSimVehicleInterface.h"

#include <limits>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "std_msgs/String.h"

ROSSimVehicleInterface::ROSSimVehicleInterface(ros::NodeHandle& nh, VehicleInfo info) :
    nh(nh),
    info(info),
    listener(buffer)
{
    std::vector<std::string> data = info.getModuleNamesOfType("DataBroadcaster");
    if(data.size() > 0)
    {
        dataSub = nh.subscribe("vehicles/" + info.getName() + "/" + data[0] + "/data", 1, &ROSSimVehicleInterface::receiveData, this);
    }
    else
    {
        ROS_FATAL("No DataBroadcaster module in vehicle");
    }

    goalPub = nh.advertise<std_msgs::String>("planner/" + info.getName() + "/goal", 1, true);
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

void ROSSimVehicleInterface::getData()
{

}

void ROSSimVehicleInterface::registerDataCallback(std::function<void(const PlannerData&)> cb)
{
    dataCallbacks.push_back(cb);
}

void ROSSimVehicleInterface::receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg)
{
    double time = msg->time.toSec();
    VehiclePose pose(msg->x, msg->y, msg->h);
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
    VehiclePose pose(std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN());

    geometry_msgs::TransformStamped transformMsg;
    tf2::Stamped<tf2::Transform> transform;
	try
    {
        if(buffer.canTransform(info.getName(), "world_ned", ros::Time(0), ros::Duration(10.0)))
        {
            transformMsg = buffer.lookupTransform("world_ned", info.getName(), ros::Time(0));
            tf2::fromMsg(transformMsg, transform);
            pose = VehiclePose((double)(transform.getOrigin().getX()),
                           (double)(transform.getOrigin().getY()),
                           (double)(transform.getOrigin().getZ())); 
        }
        else
        {
            ROS_ERROR("No valid transform available");
        }
		
	}
	catch(tf2::TransformException ex)
	{
	    ROS_ERROR("%s",ex.what());
	}

    return pose;
}