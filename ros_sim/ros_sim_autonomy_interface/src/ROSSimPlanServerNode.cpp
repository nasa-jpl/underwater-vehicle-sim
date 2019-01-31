#include "ros/ros.h"

#include "underwater_planner/Planner.h"
#include "underwater_planner/PlanDispatcher.h"
#include "ros_sim_plan_server/ROSSimPlanServer.h"
#include "ros_sim_plan_server/ROSSimVentActionFactory.h"
#include "ros_sim_plan_server/ROSSimVehicleInterface.cpp"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/NestedBinVentPlanner.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"
#include "vent_planner/DirectionSetVentPlanner.h"

#include "ros_sim_plan_server/controllers/PointPathController.h"

#include "data_server/GetLatestData.h"

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

    std::string plannerType;
    if(!nhPriv.getParam("type", plannerType))
    {
        ROS_FATAL("Parameter \"%s/type\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    //Wait until the simulation starts to proceed
    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
    vehicleInfoClient.waitForExistence();

    underwater_vehicle_msgs::GetVehicleInfo getInfo;
    vehicleInfoClient.call(getInfo);
    VehicleInfo info(getInfo);

    std::unique_ptr<PlanDispatcher> dispatcher(new PlanDispatcher());
    std::unique_ptr<VentActionFactory> factory(new ROSSimVentActionFactory(info));
    std::unique_ptr<VehicleInterface> interface(new ROSSimVehicleInterface(info));
    std::unique_ptr<Planner> planner;

    if(plannerType == "SurfaceGradient")
    {
        SurfaceGradientVentPlanner::Parameters parameters;
        nhPriv.getParam("fail_time", parameters.failTime);
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);
        nhPriv.getParam("detection_threshold", parameters.detectionThreshold);
        nhPriv.getParam("gradient_radius", parameters.gradientCalcRadius);
        nhPriv.getParam("gradient_threshold", parameters.gradientThreshold);
        nhPriv.getParam("max_follow_distance", parameters.gradientMaxFollowDistance);
        nhPriv.getParam("min_follow_distance", parameters.gradientMinFollowDistance);
        nhPriv.getParam("gradient_window", parameters.gradientWindow);

        planner.reset(new SurfaceGradientVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
    }
    else if(plannerType == "NestedBin")
    {
        NestedBinVentPlanner::Parameters parameters;
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);
        nhPriv.getParam("inital_spacing", parameters.initialSpacing);
        nhPriv.getParam("final_spacing", parameters.finalSpacing);
        nhPriv.getParam("fail_time", parameters.failTime);
        planner.reset(new NestedBinVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
    }
    else if(plannerType == "DirectionSet")
    {
        DirectionSetVentPlanner::Parameters parameters;
        nhPriv.getParam("fail_time", parameters.failTime);
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);
        nhPriv.getParam("detection_threshold", parameters.detectionThreshold);
        nhPriv.getParam("min_leg_length", parameters.minLegLength);
        nhPriv.getParam("max_leg_length", parameters.maxLegLength);
        nhPriv.getParam("leg_section_length", parameters.legSectionLength);
        nhPriv.getParam("new_max_threshold", parameters.newMaxThreshold);
        nhPriv.getParam("num_sections_threshold", parameters.numSectionsThreshold);

        planner.reset(new DirectionSetVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
    }

    std::unique_ptr<PointPathController> pointPathController(new PointPathController(info));
      
    ROSSimPlanServer server(std::move(dispatcher), std::move(planner));


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

    bool plannerCompleted = false;
    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        server.update();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}