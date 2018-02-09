#ifndef YOYO_POINT_PATH_ACTION_H
#define YOYO_POINT_PATH_ACTION_H

#include "ros/ros.h"

#include <vector>
#include <memory>
#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Action.h"
#include "planner_framework/ActionExecutor.h"

class PointPathAction : public Action, public std::enable_shared_from_this<PointPathAction>
{
public:
    PointPathAction(ActionExecutor<PointPathAction>& executor,
                        const double targetHorizontalVelocity, 
                        const double targetRotationalVelocity,
                        const double targetSlope,
                        const double upperDepth,
                        const double lowerDepth,
                        const std::vector<tf::Vector3>& points);

    PointPathAction(ActionExecutor<PointPathAction>& executor,
                        const double targetHorizontalVelocity, 
                        const double targetRotationalVelocity,
                        const std::vector<tf::Vector3>& points);

    PointPathAction(const PointPathAction& action);

    ~PointPathAction() {}

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

    /**
    *Resets this action to a state as if it has not been executed.
    */
    void reset();

    void cancel();

    void setCurrentPoint(const int point);
    const int getCurrentPoint();

    void setGoingUp(const bool goingUp);
    const bool getGoingUp();
    
    void addPointReachedTime(const ros::Time& time);
    const std::vector<ros::Time>& getPointReachedTimes();


public:
    const double targetHorizontalVelocity;
    const double targetRotationalVelocity;
    const std::vector<tf::Vector3> points;

    const bool yoyo;
    const double targetSlope;
    const double upperDepth;
    const double lowerDepth;

private:
    ActionExecutor<PointPathAction>& executor;

    int currentPoint;
    bool goingUp;
    std::vector<ros::Time> pointReachedTimes;
};

#endif