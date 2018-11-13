#ifndef YOYO_POINT_PATH_ACTION_H
#define YOYO_POINT_PATH_ACTION_H

#include <vector>
#include <memory>

#include "planner_framework/VehiclePose.h"
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
                    const bool yoyo,
                    const double upperDepth,
                    const double lowerDepth,
                    const std::vector<VehiclePose>& points,
                    const ReplanType replanType,
                    const double periodicReplanValue);

    PointPathAction(const PointPathAction& action);

    ~PointPathAction() {}

    std::shared_ptr<Action> clone() const override;

    /**
    *Executes the action using the provided executor
    */
    void executeAction() override;

    /**
    * Allows the action to trigger a replan
    */
    bool triggerReplan() override;

    /**
    * Monitors the state of the action and updates it as needed
    */
    void monitor() override;

    /**
    *Resets this action to a state as if it has not been executed.
    */
    void reset() override;

    void cancel() override;

    void setCurrentPoint(const int point);
    const int getCurrentPoint() const;

    void setGoingUp(const bool goingUp);
    const bool getGoingUp();

    void setInterruptPoint(VehiclePose point);
    void disableInterruptPoint();
    VehiclePose& getInterruptPoint();
    bool getDoInterruptPoint();

    void addPointReachedTime(const double time);
    const std::vector<double>& getPointReachedTimes();
    

    double getTargetHorizontalVelocity() const;
    double getTargetRotationalVelocity() const;
    std::vector<VehiclePose> getPoints() const;

    bool getYoyo() const;
    double getTargetSlope() const;
    double getUpperDepth() const;
    double getLowerDepth() const;
    ReplanType getReplanType() const;
    double getPeriodicReplanValue() const;

private:
    std::unique_ptr<ActionExecutor<PointPathAction>> executor;

    //Parameters
    const double targetHorizontalVelocity;
    const double targetRotationalVelocity;
    const std::vector<VehiclePose> points;

    const bool yoyo;
    const double targetSlope;
    const double upperDepth;
    const double lowerDepth;
    const ReplanType replanType;
    const double periodicReplanValue;

    /**
    * Set to true when interrupted to indicate that the interruptPoint is the next point
    */
    bool doInterruptPoint;
    
    /**
    * Indicates the point at which the vehicle was interrupted in order to command it back to that point when resumed
    */
    VehiclePose interruptPoint;

    int currentPoint;
    bool goingUp;
    
    std::vector<double> pointReachedTimes;
};

#endif