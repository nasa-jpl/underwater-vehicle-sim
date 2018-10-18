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
    //If more or complex ReplanTypes are needed we might want to 
    //look at another method of implementing them, i.e. callbacks, class
    enum ReplanType {NONE, ON_POINT_REACHED, ON_YOYO_TURN, PERIODIC_TIME, PERIODIC_DISTANCE};

    PointPathAction(std::unique_ptr<ActionExecutor<PointPathAction>> executor,
                    const double targetHorizontalVelocity, 
                    const double targetRotationalVelocity,
                    const double targetSlope,
                    const double upperDepth,
                    const double lowerDepth,
                    const std::vector<tf::Vector3>& points,
                    const ReplanType replanType,
                    const double periodicReplanValue);

    PointPathAction(std::unique_ptr<ActionExecutor<PointPathAction>> executor,
                    const double targetHorizontalVelocity, 
                    const double targetRotationalVelocity,
                    const double targetSlope,
                    const std::vector<tf::Vector3>& points,
                    const ReplanType replanType,
                    const double periodicReplanValue);

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
    const int getCurrentPoint() const;

    void setGoingUp(const bool goingUp);
    const bool getGoingUp();

    void setInterruptPoint(tf::Vector3 point);
    void disableInterruptPoint();
    tf::Vector3& getInterruptPoint();
    bool getDoInterruptPoint();

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
    const ReplanType replanType;
    const double periodicReplanValue;

private:
    std::unique_ptr<ActionExecutor<PointPathAction>> executor;

    /**
    * Set to true when interrupted to indicate that the interruptPoint is the next point
    */
    bool doInterruptPoint;
    
    /**
    * Indicates the point at which the vehicle was interrupted in order to command it back to that point when resumed
    */
    tf::Vector3 interruptPoint;

    int currentPoint;
    bool goingUp;
    
    std::vector<ros::Time> pointReachedTimes;
};

#endif