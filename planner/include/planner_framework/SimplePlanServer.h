#ifndef SIMPLE_PLAN_SERVER_H
#define SIMPLE_PLAN_SERVER_H

#include "planner_framework/Planner.h"
#include "planner_framework/PlanDispatcher.h"

#include "ros/ros.h"

class SimplePlanServer
{
public:
	SimplePlanServer(ros::NodeHandle nh, std::unique_ptr<PlanDispatcher> planDispatcher, std::unique_ptr<Planner> planner);
	SimplePlanServer(SimplePlanServer&& other);
	~SimplePlanServer() {}

	void update();

private:
	std::unique_ptr<PlanDispatcher> planDispatcher;
	std::unique_ptr<Planner> planner;
    ros::NodeHandle nh;

    const ros::Publisher clockSpeedPub;
    float speedUpFactor;
};

#endif