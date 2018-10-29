#include "ros/ros.h"

#include "planner_framework/Planner.h"
#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/PlanDispatcher.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/actions/SimVentActionFactory.h"
#include "vent_planner/NestedSpiralVentPlanner.h"
#include "vent_planner/NestedBinVentPlanner.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"
#include "vent_planner/DirectionSetVentPlanner.h"

#include "data_server/GetLatestData.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vent_planner");
    ros::NodeHandle nh;

    float loopHertz;
    if(!nh.getParam("planner/hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"planner/hertz\" not present in the parameter server.");
        exit(1);
    }

    std::string plannerType;
    if(!nh.getParam("planner/type", plannerType))
    {
        ROS_FATAL("Parameter \"planner/type\" not present in the parameter server.");
        exit(1);
    }

    std::vector<std::string> vehicleNames;
    if(!nh.getParam("vehicles/names", vehicleNames))
    {
        ROS_FATAL("Parameter \"vehicles/names\" not present in the parameter server.");
        exit(1);
    }

    std::vector<SimplePlanServer> servers;

    //Wait until the simulation starts to proceed
    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("vehicles/get_info");
    vehicleInfoClient.waitForExistence();

    for(auto& name : vehicleNames)
    {
        underwater_vehicle_msgs::GetVehicleInfo getInfo;
        getInfo.request.name = name;
        vehicleInfoClient.call(getInfo);

        VehicleInfo info(getInfo);
        
        std::unique_ptr<PlanDispatcher> dispatcher(new PlanDispatcher());
        std::unique_ptr<VentActionFactory> factory(new SimVentActionFactory(nh));
        std::unique_ptr<Planner> planner;
        if(plannerType == "SurfaceGradient")
        {
            planner.reset(new SurfaceGradientVentPlanner(nh, std::move(factory), info));
        }
        else if(plannerType == "NestedBin")
        {
            planner.reset(new NestedBinVentPlanner(nh, std::move(factory), info));
        }
        else if(plannerType == "NestedSpiral")
        {
            planner.reset(new NestedSpiralVentPlanner(nh, std::move(factory), info));
        }
        else if(plannerType == "DirectionSet")
        {
            planner.reset(new DirectionSetVentPlanner(nh, std::move(factory), info));
        }
        
        servers.emplace_back(nh, std::move(dispatcher), std::move(planner));
    }

    ROS_INFO("Planner Initalized");

    //Wait until valid data starts streaming
    ros::ServiceClient dataServerClient = nh.serviceClient<data_server::GetLatestData>("data_server/get_latest");
    dataServerClient.waitForExistence();
    data_server::GetLatestData srv;
    srv.request.name = vehicleNames[0];
    while(!dataServerClient.call(srv))
    {
       ros::WallDuration sleepDuration(1.0);
       sleepDuration.sleep();
    }
    
    bool plannersCompleted = false;
    ros::Rate r(loopHertz);
    while(!plannersCompleted && ros::ok())
    {
        bool updatePlannersCompleted = true;
        for(auto& server : servers)
        {
            updatePlannersCompleted = false;
            server.update();
        }
        plannersCompleted = updatePlannersCompleted;

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}