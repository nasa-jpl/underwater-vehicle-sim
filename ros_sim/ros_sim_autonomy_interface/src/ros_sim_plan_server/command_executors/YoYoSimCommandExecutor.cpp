#include "ros_sim_plan_server/command_executors/YoYoSimCommandExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToZ.h"

#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/planner/commands/Command.h"
#include "underwater_autonomy/planner/commands/YoYoCommand.h"

using namespace underwater_autonomy;

YoYoSimCommandExecutor::YoYoSimCommandExecutor(underwater_autonomy::YoYoCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    CommandExecutor(action),
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0)
{
    propStateSub = nh.subscribe("prop_state", 1, &YoYoSimCommandExecutor::propStateCallback, this);
    poseSub = nh.subscribe("primary_navigation", 1, &YoYoSimCommandExecutor::navigationFilterCallback, this);
    goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");
}

void YoYoSimCommandExecutor::execute()
{
    ROS_INFO("ROS: Execute YoYo Command");

    //Check that we have someone listening to us
    goToZClient.waitForExistence(ros::Duration(10));
    if(!goToZClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }

    waitForPropStateSetup();

    //Creates an action goal and sends it to the action server for point path movement
    if(sendNewGoToZGoal())
    {
        action.dispatchDone();
    }

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void YoYoSimCommandExecutor::stop()
{
    underwater_vehicle_msgs::GoToZ enableMsg;
    enableMsg.request.enable = false;
    if(goToZClient.exists() &&
       goToZClient.call(enableMsg))
    {
        action.stopDone();
    }

    ROS_INFO("ROS: Stop YoYo Command");
}

bool YoYoSimCommandExecutor::triggerReplan()
{
    if(replanNextUpdate)
    {
        replanNextUpdate = false;
        lastReplanTime = action.getLatestTime();
        distanceSinceReplan = 0;
        return true;
    }

    return false;
}

void YoYoSimCommandExecutor::propStateCallback(const underwater_vehicle_msgs::PropulsionControllerState state)
{    
    if(!statePropSetup) {
        statePropSetup = true;
        prevZSeqNum = state.zSeqNum;
    }

    double targetDepth = 0;
    if(action.getGoingUp())
    {
        targetDepth = action.getUpperDepth();
    }
    else
    {
        targetDepth = action.getLowerDepth();
    }

    if(state.zComplete && 
       doubleEq(targetDepth, state.z) &&
       !state.holdDepth &&
       prevZSeqNum != state.zSeqNum)
    {
        prevZSeqNum = state.zSeqNum;
        action.setGoingUp(!action.getGoingUp());
        sendNewGoToZGoal();

        if(action.doReplan(true, action.getLatestTime() - lastReplanTime, distanceSinceReplan)) 
        {
            replanNextUpdate = true;
        }
    }
}

void YoYoSimCommandExecutor::waitForPropStateSetup()
{
    while(!statePropSetup) {
        ros::spinOnce();
    }
}

void YoYoSimCommandExecutor::monitor()
{
    if(action.getState() == Command::State::EXECUTING && 
       action.getYoYoTime() >= 0 && 
       action.getTimeRunning() >= action.getYoYoTime())
    {
        action.complete(action.getLatestTime());
        ROS_INFO("ROS: Complete YoYo Command");
    }
    else if(action.doReplan(false, action.getLatestTime() - lastReplanTime, distanceSinceReplan))
    {
        replanNextUpdate = true;
    }
}

bool YoYoSimCommandExecutor::sendNewGoToZGoal()
{
    underwater_vehicle_msgs::GoToZ goToZMsg;
    if(action.getGoingUp())
    {
        goToZMsg.request.depth = action.getUpperDepth();
    }
    else
    {
        goToZMsg.request.depth = action.getLowerDepth();
    }

    goToZMsg.request.enable = true;
    goToZMsg.request.holdDepth = false;
    goToZMsg.request.zLinearVelocity = action.getTargetVerticalVelocity();
    if(!goToZClient.call(goToZMsg))
    {
        action.fail(action.getLatestTime());
        return false;
    }

    return true;
}

void YoYoSimCommandExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
{    
    Eigen::Vector3d position(odo.pose.pose.position.x,
                             odo.pose.pose.position.y,
                             odo.pose.pose.position.z);

    Eigen::Quaterniond orientation(odo.pose.pose.orientation.w,
                                   odo.pose.pose.orientation.x,
                                   odo.pose.pose.orientation.y,
                                   odo.pose.pose.orientation.z);

    Eigen::Matrix<double,6,6> poseCovariance;
    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            poseCovariance(i, j) = odo.pose.covariance[(i * 6) + j];
        }
    }


    Eigen::Vector3d linearVelocity(odo.twist.twist.linear.x,
                                   odo.twist.twist.linear.y,
                                   odo.twist.twist.linear.z);
    Eigen::Vector3d angularVelocity(odo.twist.twist.angular.x,
                                    odo.twist.twist.angular.y,
                                    odo.twist.twist.angular.z);

    Eigen::Matrix<double,6,6> twistCovariance;
    for(unsigned int i = 0; i < 6; i++)
    {
        for(unsigned int j = 0; j < 6; j++)
        {
            twistCovariance(i, j) = odo.twist.covariance[(i * 6) + j];
        }
    }

    //Update the distance since replanning
    Eigen::Vector3d zeroedPosition = position;
    Eigen::Vector3d zeroedCurrentPosition = currentPose.getPosition();
    zeroedPosition[0] = 0;
    zeroedPosition[1] = 0;
    zeroedCurrentPosition[0] = 0;
    zeroedCurrentPosition[1] = 0;
    distanceSinceReplan += (zeroedPosition - zeroedCurrentPosition).norm();

    currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);

    if((action.getState() == Command::State::DISPATCHED ||
        action.getState() == Command::State::EXECUTING ||
        action.getState() == Command::State::PAUSING ||
        action.getState() == Command::State::COMPLETING) &&
        !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(action.getLatestTime());
    }  
}

bool YoYoSimCommandExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}