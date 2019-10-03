#include "ros/ros.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"
#include "underwater_autonomy/util/BoxOperationRegion.h"
#include "underwater_autonomy/planner/actions/ActionFactory.h"
#include "underwater_autonomy/planner/SingleActionPlanner.h"

#include "ros_sim_plan_server/ROSSimPlanServer.h"
#include "ros_sim_plan_server/ROSSimActionFactory.h"
#include "ros_sim_plan_server/ROSSimVehicleInterface.cpp"

#include "vent_planner/NestedBinVentPlanner.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"
#include "vent_planner/DirectionSetVentPlanner.h"
#include "vent_planner/WaypointsPlanner.h"

#include "data_server/GetLatestData.h"

using namespace underwater_autonomy;
using namespace vent_planner;

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

    ROSSimActionFactory factory(info);
    ROSSimVehicleInterface interface(info);
    std::unique_ptr<Planner> planner;

    if(plannerType == "SurfaceGradient")
    {
        SurfaceGradientVentPlanner::Parameters parameters;
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);
        nhPriv.getParam("detection_threshold", parameters.detectionThreshold);
        nhPriv.getParam("gradient_radius", parameters.gradientCalcRadius);
        nhPriv.getParam("gradient_threshold", parameters.gradientThreshold);
        nhPriv.getParam("max_follow_distance", parameters.gradientMaxFollowDistance);
        nhPriv.getParam("min_follow_distance", parameters.gradientMinFollowDistance);
        nhPriv.getParam("gradient_window", parameters.gradientWindow);

        planner.reset(new SurfaceGradientVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "NestedBin")
    {
        NestedBinVentPlanner::Parameters parameters;
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);
        nhPriv.getParam("inital_spacing", parameters.initialSpacing);
        nhPriv.getParam("final_spacing", parameters.finalSpacing);
        nhPriv.getParam("target_data", parameters.targetData);
        nhPriv.getParam("detection_threshold", parameters.detectionThreshold);

        nhPriv.getParam("lawnmower_data_range", parameters.lawnmowerDataRange);

        if(nhPriv.hasParam("yoyo_min_depth") && nhPriv.hasParam("yoyo_max_depth"))
        {
            nhPriv.getParam("yoyo_min_depth", parameters.yoyoMinDepth);
            nhPriv.getParam("yoyo_max_depth", parameters.yoyoMaxDepth);
            parameters.yoyoDuringSpiral = true;
        }
        else
        {
            parameters.yoyoMinDepth = 0;
            parameters.yoyoMaxDepth = 0;
            parameters.yoyoDuringSpiral = false;
        }
        nhPriv.getParam("target_horizontal_velocity", parameters.targetHorizontalVelocity);
        nhPriv.getParam("target_vertical_velocity", parameters.targetVerticalVelocity);

        double startX, startY, startZ;
        nhPriv.getParam("spiral_start_x", startX);
        nhPriv.getParam("spiral_start_y", startY);
        nhPriv.getParam("spiral_start_z", startZ);
        parameters.startLocation = Eigen::Vector3d(startX, startY, startZ);

        double targetX, targetY;
        if(nhPriv.hasParam("sim_target_x") && nhPriv.hasParam("sim_target_y"))
        {
            parameters.hasSimTarget = true;
            nhPriv.getParam("sim_target_x", targetX);
            nhPriv.getParam("sim_target_y", targetY);
            parameters.simTarget = Eigen::Vector3d(targetX, targetY, 0);
        }
        else
        {
            parameters.hasSimTarget = false;
        }

        double minX, minY, minZ, maxX, maxY, maxZ;
        nhPriv.getParam("operation_region_min_x", minX);
        nhPriv.getParam("operation_region_min_y", minY);
        nhPriv.getParam("operation_region_min_z", minZ);

        nhPriv.getParam("operation_region_max_x", maxX);
        nhPriv.getParam("operation_region_max_y", maxY);
        nhPriv.getParam("operation_region_max_z", maxZ);
        parameters.operationRegion = std::unique_ptr<OperationRegion>(new BoxOperationRegion(minX, minY, minZ, maxX, maxY, maxZ));

        nhPriv.getParam("sonar_data", parameters.sonarData);
        nhPriv.getParam("sonar_depth_enabled", parameters.sonarDepthEnabled);

        planner.reset(new NestedBinVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "DirectionSet")
    {
        DirectionSetVentPlanner::Parameters parameters;
        nhPriv.getParam("spiral_spacing", parameters.spiralSpacing);

        nhPriv.getParam("detection_threshold", parameters.detectionThreshold);

        nhPriv.getParam("min_leg_length", parameters.minLegLength);
        nhPriv.getParam("max_leg_length", parameters.maxLegLength);

        nhPriv.getParam("new_max_threshold", parameters.newMaxThreshold);

        nhPriv.getParam("leg_section_length", parameters.legSectionLength);
        nhPriv.getParam("num_sections_threshold", parameters.numSectionsThreshold);

        nhPriv.getParam("target_horizontal_velocity", parameters.targetHorizontalVelocity);

        nhPriv.getParam("data_depth_range", parameters.dataDepthRange);

        if(nhPriv.hasParam("yoyo_min_depth") && nhPriv.hasParam("yoyo_max_depth"))
        {
            nhPriv.getParam("yoyo_min_depth", parameters.yoyoMinDepth);
            nhPriv.getParam("yoyo_max_depth", parameters.yoyoMaxDepth);
            parameters.yoyoDuringSpiral = true;
        }
        else
        {
            parameters.yoyoMinDepth = 0;
            parameters.yoyoMaxDepth = 0;
            parameters.yoyoDuringSpiral = false;
        }

        double startX, startY, startZ;
        nhPriv.getParam("spiral_start_x", startX);
        nhPriv.getParam("spiral_start_y", startY);
        nhPriv.getParam("spiral_start_z", startZ);
        parameters.startLocation = Eigen::Vector3d(startX, startY, startZ);

        nhPriv.getParam("target_data", parameters.targetData);

        double minX, minY, minZ, maxX, maxY, maxZ;
        nhPriv.getParam("operation_region_min_x", minX);
        nhPriv.getParam("operation_region_min_y", minY);
        nhPriv.getParam("operation_region_min_z", minZ);
        nhPriv.getParam("operation_region_max_x", maxX);
        nhPriv.getParam("operation_region_max_y", maxY);
        nhPriv.getParam("operation_region_max_z", maxZ);
        
        parameters.operationRegion = std::unique_ptr<OperationRegion>(new BoxOperationRegion(minX, minY, minZ, maxX, maxY, maxZ));

        planner.reset(new DirectionSetVentPlanner(factory, interface, parameters));
    }
    else if(plannerType == "Waypoints")
    {
        WaypointsPlanner::Parameters parameters;
        std::vector<double> waypointsX;
        std::vector<double> waypointsY;
        std::vector<double> waypointsZ;

        nhPriv.getParam("waypoints_x", waypointsX);
        nhPriv.getParam("waypoints_y", waypointsY);
        nhPriv.getParam("waypoints_z", waypointsZ);

        if(waypointsX.size() != waypointsY.size() || 
          waypointsX.size() != waypointsZ.size())
        {
            ROS_FATAL("Parameters \"%s/waypoints_x\", \"%s/waypoints_y\", and \"%s/waypoints_z\" must have the same size.", nhPriv.getNamespace().c_str(), 
                                                                                                                            nhPriv.getNamespace().c_str(), 
                                                                                                                            nhPriv.getNamespace().c_str());
            exit(1);
        }
        
        for(unsigned int i = 0; i < waypointsX.size(); i++)
        {
            parameters.waypoints.push_back(Eigen::Vector3d(waypointsX[i], waypointsY[i], waypointsZ[i]));
        }

        planner.reset(new WaypointsPlanner(factory, interface, parameters));
    }
    else if(plannerType == "SingleAction")
    {
        planner.reset(new SingleActionPlanner(factory, interface));
    }
      
    ROSSimPlanServer server(std::move(planner), interface);

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

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}