#include "planner_framework/SimplePlanServer.h"
#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

#include "std_msgs/Float64.h"

#include "ros/ros.h"

SimplePlanServer::SimplePlanServer(ros::NodeHandle nh, std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner) :
	nh(nh),
	planDispatcher(std::move(planDispatcher)),
	planner(std::move(planner)),
	clockSpeedPub(nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1))
{
	nh.param<float>("speed_up_factor", speedUpFactor, 1);
	this->planDispatcher->run();
}

SimplePlanServer::SimplePlanServer(SimplePlanServer&& other) :
	planDispatcher(std::move(other.planDispatcher)),
	planner(std::move(other.planner))
{
	this->planDispatcher->run();
}

void SimplePlanServer::update()
{
	planDispatcher->update();

	if(planDispatcher->triggerReplan())
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
			planDispatcher->setPlan(newPlan);
			planDispatcher->run();
		}	
	}
}