#include "ros_sim_behavior_server/command_executors/SampleSimCommandExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "underwater_autonomy/behaviors/commands/SampleCommand.h"

using namespace underwater_autonomy;

SampleSimCommandExecutor::SampleSimCommandExecutor(underwater_autonomy::SampleCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    CommandExecutor(action),
    vehicleInfo(vehicleInfo)
{
    std::vector<std::string> sampleModules = vehicleInfo.getModuleNamesOfType("Sample");
    if(sampleModules.size() > 0) {
        requestSampleClient = nh.serviceClient<underwater_vehicle_msgs::RequestSample>(sampleModules[0] + "/request_sample");
    }
}

void SampleSimCommandExecutor::execute()
{
    action.dispatchDone();
    ROS_INFO("ROS: Execute Sample Command");
    
    requestSampleClient.waitForExistence(ros::Duration(10));
    if(!requestSampleClient.exists())
    {
        action.fail(action.getLatestTime());
        ROS_INFO("ROS: Sample Command Failed");
    } else {
        underwater_vehicle_msgs::RequestSample requestSampleMsg;
        if(requestSampleClient.call(requestSampleMsg)) {
            action.complete(action.getLatestTime());
            ROS_INFO("ROS: Sample Command Complete");
        } else {
            action.fail(action.getLatestTime());
            ROS_INFO("ROS: Sample Command Failed");
        }
    }
}

void SampleSimCommandExecutor::stop()
{
    action.stopDone();
    ROS_INFO("ROS: Stop Sample Command");
}

void SampleSimCommandExecutor::monitor()
{}