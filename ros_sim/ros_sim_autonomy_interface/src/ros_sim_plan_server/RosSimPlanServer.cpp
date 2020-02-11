#include "ros_sim_plan_server/ROSSimPlanServer.h"

#include "underwater_autonomy/planner/Planner.h"
#include "underwater_autonomy/planner/PlanDispatcher.h"

#include "std_msgs/Float64.h"

#include "ros/ros.h"

using namespace underwater_autonomy;

ROSSimPlanServer::ROSSimPlanServer(std::unique_ptr<Planner> planner,
                                   ROSSimVehicleInterface& vehicleInterface,
                                   int cancelTimeout) :
    planner(std::move(planner)),
    vehicleInterface(vehicleInterface),
    planDispatcher(vehicleInterface, cancelTimeout)
{
    ros::NodeHandle nh;

    clockSpeedPub = nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1);
    nh.param<float>("/speed_up_factor", speedUpFactor, 1);
    
    planDispatcher.run();
}

ROSSimPlanServer::ROSSimPlanServer(ROSSimPlanServer&& other) :
    planner(std::move(other.planner)),
    vehicleInterface(other.vehicleInterface),
    planDispatcher(other.planDispatcher)
{
    planDispatcher.run();
}

std::string ROSSimPlanServer::getPlannerStatus()
{
    return plannerStatus;
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
    
        planDispatcher.setPlan(newPlan);
    }

    if(planDispatcher.getState() == PlanDispatcher::PlanDispatcherState::WAITING_FOR_CANCEL_STOPPED ||
       planDispatcher.getState() == PlanDispatcher::PlanDispatcherState::WAITING_FOR_CANCEL_RUNNING ||
       planDispatcher.getState() == PlanDispatcher::PlanDispatcherState::WAITING_FOR_CANCEL_FAIL_ACTION )
    {
        std_msgs::Float64 slowSim;
        slowSim.data = 1;
        clockSpeedPub.publish(slowSim);
    } 
    else 
    {
        std_msgs::Float64 startSim;
        startSim.data = speedUpFactor;
        clockSpeedPub.publish(startSim);
    }

    plannerStatus = planner->getPlannerStatus();
}