#include "ros/ros.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"
#include "underwater_autonomy/planner/actions/ActionFactory.h"
#include "underwater_autonomy/planner/SingleActionPlanner.h"
#include "underwater_autonomy/planner/ConfigurationFile.h"

#include "ros_sim_plan_server/ROSSimPlanServer.h"
#include "ros_sim_plan_server/ROSSimActionFactory.h"
#include "ros_sim_plan_server/ROSSimVehicleInterface.cpp"

#include "vent_planner/NestedBinVentPlanner.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"
#include "vent_planner/DirectionSetVentPlanner.h"
#include "vent_planner/WaypointsPlanner.h"

#include "navigation_planner/GoldenSelectionHomingPlanner.h"


#include "data_server/GetLatestData.h"

using namespace underwater_autonomy;
using namespace vent_planner;
using namespace navigation_planner;

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
    ros::init(argc, argv, "ros_sim_plan_server");
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
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);

    ROSSimActionFactory factory(info);
    ROSSimVehicleInterface interface(info);
    std::unique_ptr<Planner> planner;

    std::string configFilename;
    nhPriv.getParam("planner_config_file", configFilename);
    ConfigurationFile config(configFilename);

    std::string plannerType = config.readSimpleEntry<std::string>("planner_type");
    if(plannerType == "SurfaceGradient")
    {
        SurfaceGradientVentPlanner::Parameters parameters(config);
        planner.reset(new SurfaceGradientVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "NestedBin")
    {
        NestedBinVentPlanner::Parameters parameters(config);
        planner.reset(new NestedBinVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "DirectionSet")
    {
        DirectionSetVentPlanner::Parameters parameters(config);

        planner.reset(new DirectionSetVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "Waypoints")
    {
        WaypointsPlanner::Parameters parameters(config);
        planner.reset(new WaypointsPlanner(factory, interface, parameters));
    }
    else if(plannerType == "SingleAction")
    {
        planner.reset(new SingleActionPlanner(factory, interface));
    }
    else if(plannerType == "GoldenSelectionHoming")
    {
        GoldenSelectionHomingPlanner::Parameters parameters(config);
        planner.reset(new GoldenSelectionHomingPlanner(factory, interface, parameters));
    }

    ROSSimPlanServer server(std::move(planner), interface, cancelTimeout);

    // make a publisher to send planner status messages
    ros::Publisher plannerStatus_pub = nh.advertise<std_msgs::String>("plannerStatus", 1000);
    std_msgs::String plannerStatusMsg;

    ROS_INFO("Planner Initalized");

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

    ROS_INFO("Planner Started");

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        server.update();

        plannerStatusMsg.data = server.getPlannerStatus();
        plannerStatus_pub.publish(plannerStatusMsg);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
