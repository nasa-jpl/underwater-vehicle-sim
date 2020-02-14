#ifndef POINT_PATH_SIM_ACTION_EXECUTOR_H
#define POINT_PATH_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "underwater_vehicle_msgs/GoToXYComplete.h"
#include "std_msgs/Bool.h"

#include "underwater_autonomy/planner/ActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_plan_server/action_executors/SimActionExecutorFactoryMethod.h"

class PointPathSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::PointPathAction>,
                                   public SimActionExecutorFactoryMethod<PointPathSimActionExecutor>
{
public:
    PointPathSimActionExecutor(ros::NodeHandle& nh, VehicleInfo& info);
    PointPathSimActionExecutor(const PointPathSimActionExecutor&&) = delete;
    PointPathSimActionExecutor(const PointPathSimActionExecutor&) = delete;

    PointPathSimActionExecutor& operator=(PointPathSimActionExecutor&& ) = delete;
    PointPathSimActionExecutor& operator=(PointPathSimActionExecutor& ) = delete;

    ~PointPathSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    void execute(underwater_autonomy::PointPathAction& action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(underwater_autonomy::PointPathAction& action) override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(underwater_autonomy::PointPathAction& action) override;

    void stop(underwater_autonomy::PointPathAction& action) override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);
    void goToXYCompleteCallback(const underwater_vehicle_msgs::GoToXYComplete complete);

    void sendNextGoToXYGoal(underwater_autonomy::PointPathAction& action);

    void propStateCB(const std_msgs::Bool data);

    bool doubleEq(double d1, double d2);
    
private:
    VehicleInfo vehicleInfo;

    ros::Publisher goToXYPub;
    ros::ServiceClient goToXYEnableClient;
    ros::Subscriber goToXYComplete;
    
    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;

    ros::Publisher velPub;
    ros::Subscriber poseSub;
    
    underwater_autonomy::VehiclePose currentPose;

    bool gotCompleteCallback;
    double completeCallbackX;
    double completeCallbackY;
};

#endif