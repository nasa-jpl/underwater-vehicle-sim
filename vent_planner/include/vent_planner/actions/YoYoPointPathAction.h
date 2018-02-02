#ifndef YOYO_POINT_PATH_ACTION_H
#define YOYO_POINT_PATH_ACTION_H

#include "ros/ros.h"

#include <vector>
#include <memory>
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Action.h"
#include "planner_framework/ActionExecutor.h"

class YoYoPointPathAction : public Action, public std::enable_shared_from_this<YoYoPointPathAction>
{
public:
	YoYoPointPathAction(ActionExecutor<YoYoPointPathAction>& executor,
						const double targetHorizontalVelocity, 
						const double targetRotationalVelocity,
						const double targetSlope,
						const double upperDepth,
						const double lowerDepth,
						const std::vector<tf::Vector3>& points);

	YoYoPointPathAction(const YoYoPointPathAction& action);

	~YoYoPointPathAction() {}

	std::shared_ptr<Action> clone() const override;

	/**
	*Executes the action using the provided executor
	*/
	void executeAction();

	/**
	* Allows the action to trigger a replan
	*/
	bool triggerReplan();

	/**
	* Monitors the state of the action and updates it as needed
	*/
	void monitor();

	void setCurrentPoint(const int point);
	const int getCurrentPoint();
	
	void addPointReachedTime(const ros::Time& time);
	const std::vector<ros::Time>& getPointReachedTimes();


public:
	const double targetHorizontalVelocity;
	const double targetRotationalVelocity;
	const double targetSlope;
	const double upperDepth;
	const double lowerDepth;
	const std::vector<tf::Vector3> points;

private:
	ActionExecutor<YoYoPointPathAction>& executor;

	int currentPoint;
	std::vector<ros::Time> pointReachedTimes;

};

#endif