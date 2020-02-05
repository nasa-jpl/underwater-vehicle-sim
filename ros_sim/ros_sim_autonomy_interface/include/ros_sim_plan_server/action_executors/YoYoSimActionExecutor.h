#ifndef YOYO_SIM_ACTION_EXECUTOR_H
#define YOYO_SIM_ACTION_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "underwater_vehicle_msgs/GoToZComplete.h"

#include "underwater_autonomy/planner/ActionExecutor.h"
#include "underwater_autonomy/planner/actions/YoYoAction.h"

#include "underwater_autonomy/util/VehiclePose.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

#include "ros_sim_plan_server/action_executors/SimActionExecutorFactoryMethod.h"

class YoYoSimActionExecutor : public underwater_autonomy::ActionExecutor<underwater_autonomy::YoYoAction>,
                              public SimActionExecutorFactoryMethod<YoYoSimActionExecutor>
{
public:
    YoYoSimActionExecutor(ros::NodeHandle& nh, VehicleInfo& vehicleInfo);

    YoYoSimActionExecutor(const YoYoSimActionExecutor&&) = delete;
    YoYoSimActionExecutor(const YoYoSimActionExecutor&) = delete;

    YoYoSimActionExecutor& operator=(YoYoSimActionExecutor&& ) = delete;
    YoYoSimActionExecutor& operator=(YoYoSimActionExecutor& ) = delete;

    ~YoYoSimActionExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    bool execute(std::shared_ptr<underwater_autonomy::YoYoAction> action) override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor(std::shared_ptr<underwater_autonomy::YoYoAction> action) override;

    /**
    * Allows the yoyo action to trigger a replan in the ros simulation 
    */
    bool triggerReplan(std::shared_ptr<underwater_autonomy::YoYoAction> action) override;

    void cancel(std::shared_ptr<underwater_autonomy::YoYoAction> action) override;

private:
    void navigationFilterCallback(const nav_msgs::Odometry odo);
    void goToZCompleteCallback(const underwater_vehicle_msgs::GoToZComplete complete);

    void sendNewGoToZGoal(std::shared_ptr<underwater_autonomy::YoYoAction> action);

    bool doubleEq(double d1, double d2);

private:
    VehicleInfo vehicleInfo;

    ros::Publisher goToZPub;
    ros::Publisher goToZEnablePub;
    ros::Subscriber goToZComplete;

    ros::ServiceClient propStateClient;
    
    bool replanNextUpdate;
    ros::Time lastReplan;
    double distanceSinceReplan;

    tf2::Vector3 lastLocation;
    ros::Publisher velPub;
    ros::Subscriber poseSub;
    underwater_autonomy::VehiclePose currentPose;

    bool gotCompleteCallback;
    double completeCallbackZ;
    bool completeCallbackHoldDepth;
};

#endif