#include "ros_sim_plan_server/ROSSimVehicleInterface.h"

#include <limits>

#include "ros/ros.h"

ROSSimVehicleInterface::ROSSimVehicleInterface(ros::NodeHandle& nh, VehicleInfo info) :
    nh(nh),
    info(info)
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
    tf::StampedTransform transform;

    VehiclePose pose(std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN());
    try
    {
        if(listener.waitForTransform("/world", "/" + info.getName(),
                                  ros::Time(0), ros::Duration(5.0)))
        {
           listener.lookupTransform("/world", "/" + info.getName(),  
                                 ros::Time(0), transform);

            pose = VehiclePose((double)(transform.getOrigin().getX()),
                               (double)(transform.getOrigin().getY()),
                               (double)(transform.getOrigin().getZ())); 
        }
        else
        {
            ROS_ERROR("No valid transform available");
        }
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    return pose;
}