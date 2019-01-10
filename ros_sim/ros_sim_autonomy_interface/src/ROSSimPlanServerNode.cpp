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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_sim_plan_server");
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
    if(!nh.getParam("underwater_vehicle_sim/vehicles/names", vehicleNames))
    {
        ROS_FATAL("Parameter \"underwater_vehicle_sim/vehicles/names\" not present in the parameter server.");
        exit(1);
    }

    std::vector<ROSSimPlanServer> servers;

    std::vector<std::unique_ptr<PointPathController>> pointPathControllers;

    //Wait until the simulation starts to proceed
    ros::ServiceClient vehicleInfoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("underwater_vehicle_sim/vehicles/get_info");
    vehicleInfoClient.waitForExistence();

    for(auto& name : vehicleNames)
    {
        underwater_vehicle_msgs::GetVehicleInfo getInfo;
        getInfo.request.name = name;
        vehicleInfoClient.call(getInfo);

        VehicleInfo info(getInfo);

        std::unique_ptr<PlanDispatcher> dispatcher(new PlanDispatcher());
        std::unique_ptr<VentActionFactory> factory(new ROSSimVentActionFactory(nh, info));
        std::unique_ptr<VehicleInterface> interface(new ROSSimVehicleInterface(nh, info));
        std::unique_ptr<Planner> planner;
        if(plannerType == "SurfaceGradient")
        {
            SurfaceGradientVentPlanner::Parameters parameters;
            nh.getParam("planner/fail_time", parameters.failTime);
            nh.getParam("planner/spiral_spacing", parameters.spiralSpacing);
            nh.getParam("planner/detection_threshold", parameters.detectionThreshold);
            nh.getParam("planner/gradient_radius", parameters.gradientCalcRadius);
            nh.getParam("planner/gradient_threshold", parameters.gradientThreshold);
            nh.getParam("planner/max_follow_distance", parameters.gradientMaxFollowDistance);
            nh.getParam("planner/min_follow_distance", parameters.gradientMinFollowDistance);
            nh.getParam("planner/gradient_window", parameters.gradientWindow);

            planner.reset(new SurfaceGradientVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
        }
        else if(plannerType == "NestedBin")
        {
            NestedBinVentPlanner::Parameters parameters;
            nh.getParam("planner/spiral_spacing", parameters.spiralSpacing);
            nh.getParam("planner/inital_spacing", parameters.initialSpacing);
            nh.getParam("planner/final_spacing", parameters.finalSpacing);
            nh.getParam("planner/fail_time", parameters.failTime);
            planner.reset(new NestedBinVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
        }
        else if(plannerType == "DirectionSet")
        {
            DirectionSetVentPlanner::Parameters parameters;
            nh.getParam("planner/fail_time", parameters.failTime);
            nh.getParam("planner/spiral_spacing", parameters.spiralSpacing);
            nh.getParam("planner/detection_threshold", parameters.detectionThreshold);
            nh.getParam("planner/min_leg_length", parameters.minLegLength);
            nh.getParam("planner/max_leg_length", parameters.maxLegLength);
            nh.getParam("planner/leg_section_length", parameters.legSectionLength);
            nh.getParam("planner/new_max_threshold", parameters.newMaxThreshold);
            nh.getParam("planner/num_sections_threshold", parameters.numSectionsThreshold);

            planner.reset(new DirectionSetVentPlanner(std::move(factory), std::move(interface), std::move(parameters)));
        }
        
        std::unique_ptr<PointPathController> pointPathController(new PointPathController(nh, info));
        pointPathControllers.push_back(std::move(pointPathController));
      
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