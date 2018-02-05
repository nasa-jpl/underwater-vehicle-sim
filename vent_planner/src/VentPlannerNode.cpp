#include "ros/ros.h"

#include "planner_framework/Planner.h"
#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/PlanDispatcher.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/SimVentActionFactory.h"
#include "vent_planner/VentPlanner.h"

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

    std::vector<std::string> vehicleNames;
    if(!nh.getParam("vehicles/names", vehicleNames))
    {
        ROS_FATAL("Parameter \"vehicles/names\" not present in the parameter server.");
        exit(1);
    }

    std::vector<SimplePlanServer> servers;

    for(auto& name : vehicleNames)
    {

        std::unique_ptr<PlanDispatcher> dispatcher(new PlanDispatcher());
        std::unique_ptr<VentActionFactory> factory(new SimVentActionFactory(nh));

        std::unique_ptr<Planner> planner(new VentPlanner(nh, std::move(factory), name));
        servers.emplace_back(std::move(dispatcher), std::move(planner));
    }

    ros::Rate r(loopHertz);
    while(ros::ok())
    {
        for(auto& server : servers)
        {
            server.update();
        }
        
        r.sleep();
    }
    return 0;
}