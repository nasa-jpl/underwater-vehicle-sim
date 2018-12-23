#ifndef POINT_PATH_CONTROLLER_H
#define POINT_PATH_CONTROLLER_H

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "ros_sim_plan_server/PointPathRosAction.h"

#include "tf/LinearMath/Vector3.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/VehicleData.h"

#include "vehicle_auto_control/GoToXYRosAction.h"
#include "vehicle_auto_control/GoToZRosAction.h"

class PointPathController
{
public:
    PointPathController(ros::NodeHandle nh, 
                        VehicleInfo vehicleInfo);
    
    ~PointPathController() {}

private:
    void sendAllGoals(void);
    void pointPathUpdate(void);
    void yoyoUpdate(void);

    void goalCB(void);
    void preemptCB(void);
    
    void sendFeedback(void);

    void getVehicleData(const underwater_vehicle_msgs::VehicleData data);

    void goToXYActive(void);
    void goToXYFeedback(const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback);
    void goToXYDone(const actionlib::SimpleClientGoalState& state,
                    const vehicle_auto_control::GoToXYRosResultConstPtr& result);
    void sendXYGoal(const double x, const double y);

    void goToZActive(void);
    void goToZFeedback(const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback);
    void goToZDone(const actionlib::SimpleClientGoalState& state,
                   const vehicle_auto_control::GoToZRosResultConstPtr& result);
    void sendZGoal(const double z);

private:
    actionlib::SimpleActionServer<ros_sim_plan_server::PointPathRosAction> pointPathServer;
    actionlib::SimpleActionClient<vehicle_auto_control::GoToXYRosAction> goToXYClient;
    actionlib::SimpleActionClient<vehicle_auto_control::GoToZRosAction> goToZClient;

    ros::NodeHandle nh;

    VehicleInfo vehicleInfo;

    std::vector<tf::Vector3> pathPoints;
    unsigned int currentPoint;

    bool xyDone;
    bool zDone;
    //Shared Parameters
    bool yoyo;
    int upperDepth;
    int lowerDepth;

    //State variables
    bool goingUp;

    ros::Subscriber dataSub;
    double latestSonarDepth;
    double latestVehicleDepth;

    bool newGoalAccepted;
};

#endif
