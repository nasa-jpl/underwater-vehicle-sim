#include "ros_sim_plan_server/ROSSimPlanServer.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"

#include "std_msgs/Float64.h"

#include "ros/ros.h"

using namespace underwater_autonomy;

ROSSimPlanServer::ROSSimPlanServer(std::unique_ptr<Planner> planner,
                                   ROSSimVehicleInterface& vehicleInterface) :
    planner(std::move(planner)),
    vehicleInterface(vehicleInterface),
    planDispatcher(vehicleInterface)
{
    ros::NodeHandle nh;

    clockSpeedPub = nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1);
    nh.param<float>("/speed_up_factor", speedUpFactor, 1);
    
    planDispatcher.run();
}

ROSSimPlanServer::ROSSimPlanServer(ROSSimPlanServer&& other) :

    planDispatcher(other.planDispatcher),
    vehicleInterface(other.vehicleInterface),
    planner(std::move(other.planner))
{
    planDispatcher.run();
}

void ROSSimPlanServer::update()
{
    planDispatcher.update();

    if(planDispatcher.triggerReplan())
    {
        std_msgs::Float64 slowSim;
        slowSim.data = 1;
        clockSpeedPub.publish(slowSim);

        std::shared_ptr<Plan> newPlan = planner->plan();
    
        std_msgs::Float64 startSim;
        startSim.data = speedUpFactor;
        clockSpeedPub.publish(startSim);

        if(newPlan)
        {
            planDispatcher.setPlan(newPlan);
            planDispatcher.run();
        }
    }
}