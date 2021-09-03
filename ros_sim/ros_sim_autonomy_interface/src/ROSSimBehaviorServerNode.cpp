#include "ros/ros.h"

#include "std_msgs/String.h"

#include "underwater_autonomy/planner/Behavior.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"
#include "underwater_autonomy/planner/SingleCommandBehavior.h"
#include "underwater_autonomy/util/ConfigurationFile.h"
#include "underwater_autonomy/planner/BehaviorFactory.h"
#include "underwater_autonomy/planner/WaypointsBehavior.h"

#include "ros_sim_behavior_server/ROSSimBehaviorServer.h"
#include "ros_sim_autonomy_interface/ROSSimVehicleInterface.h"

#include "vent_behaviors/NestedBinVentBehavior.h"
#include "vent_behaviors/SurfaceGradientVentBehavior.h"
#include "vent_behaviors/DirectionSetVentBehavior.h"

#include "navigation_behaviors/GoldenSelectionHomingBehavior.h"
#include "navigation_behaviors/NonLinearFilterHomingBehavior.h"

#include "explore_behaviors/OutAndBackExploreBehavior.h"
#include "explore_behaviors/InWaterTestBehavior.h"

#include "ros_sim_behavior_server/command_executors/YoYoSimCommandExecutor.h"
#include "ros_sim_behavior_server/command_executors/HoldDepthSimCommandExecutor.h"
#include "ros_sim_behavior_server/command_executors/WaypointsSimCommandExecutor.h"
#include "ros_sim_behavior_server/command_executors/FollowHeadingSimCommandExecutor.h"
#include "ros_sim_behavior_server/command_executors/CircleSimCommandExecutor.h"
#include "ros_sim_behavior_server/command_executors/SampleSimCommandExecutor.h"

using namespace underwater_autonomy;
using namespace vent_behaviors;
using namespace navigation_behaviors;
using namespace explore_behaviors;

bool dataStarted = false;
bool navStarted = false;

void receiveData(const underwater_vehicle_msgs::VehicleData::ConstPtr& msg)
{
    dataStarted = true;
}

void receiveNav(const nav_msgs::Odometry::ConstPtr& msg)
{
    navStarted = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_behavior_server");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    float loopHertz;
    if(!nhPriv.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    int cancelTimeout = 60; //Default to one minute
    nhPriv.getParam("cancel_timeout", cancelTimeout);

    //Wait until the simulation starts to proceed
    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    vehicleInfoClient.waitForExistence();
    underwater_vehicle_msgs::GetVehicleInfo getInfo;

    std::size_t found = nh.getNamespace().find_last_of("/");
    getInfo.request.name = nh.getNamespace().substr(found+1);
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);

    std::shared_ptr<ROSSimVehicleInterface> interface = std::make_shared<ROSSimVehicleInterface>(info);

    YoYoCommand::setExecutorCreateFunction(std::bind(&YoYoSimCommandExecutor::create, std::placeholders::_1, nh, info));
    HoldDepthCommand::setExecutorCreateFunction(std::bind(&HoldDepthSimCommandExecutor::create, std::placeholders::_1, nh, info));
    WaypointsCommand::setExecutorCreateFunction(std::bind(&WaypointsSimCommandExecutor::create, std::placeholders::_1, nh, info));
    FollowHeadingCommand::setExecutorCreateFunction(std::bind(&FollowHeadingSimCommandExecutor::create, std::placeholders::_1, nh, info));
    CircleCommand::setExecutorCreateFunction(std::bind(&CircleSimCommandExecutor::create, std::placeholders::_1, nh, info));
    SampleCommand::setExecutorCreateFunction(std::bind(&SampleSimCommandExecutor::create, std::placeholders::_1, nh, info));


    std::string configFilename;
    nhPriv.getParam("planner_config_file", configFilename);
    ConfigurationFile config(configFilename);

    BehaviorFactory::registerBehavior("SurfaceGradient", &SurfaceGradientVentBehavior::create);
    BehaviorFactory::registerBehavior("NestedBin", &NestedBinVentBehavior::create);
    BehaviorFactory::registerBehavior("DirectionSet", &DirectionSetVentBehavior::create);
    BehaviorFactory::registerBehavior("Waypoints", &WaypointsBehavior::create);
    BehaviorFactory::registerBehavior("SingleCommand", &SingleCommandBehavior::create);
    BehaviorFactory::registerBehavior("GoldenSelectionHoming", &GoldenSelectionHomingBehavior::create);
    BehaviorFactory::registerBehavior("NonLinearFilterHoming", &NonLinearFilterHomingBehavior::create);
    BehaviorFactory::registerBehavior("OutAndBackExplore", &OutAndBackExploreBehavior::create);
    BehaviorFactory::registerBehavior("InWaterTest", &InWaterTestBehavior::create);

    std::string plannerType = config.readSimpleEntry<std::string>("planner_type");
    std::unique_ptr<Behavior> behavior = BehaviorFactory::create(plannerType, interface, config);

    ROSSimBehaviorServer server(std::move(behavior), interface);

    ROS_INFO("Behavior Initalized");

    //Wait until valid data starts streaming
    ros::Subscriber dataSub;
    std::vector<std::string> data = info.getModuleNamesOfType("DataBroadcaster");

    if(data.size() > 0)
    {
        dataSub = nh.subscribe(data[0] + "/data", 1, &receiveData);
    }

    //Wait until valid navigation starts streaming
    ros::Subscriber navSub = nh.subscribe("primary_navigation", 1, &receiveNav);

    while(!(dataStarted && navStarted))
    {
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Behavior Started");

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        server.update();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
