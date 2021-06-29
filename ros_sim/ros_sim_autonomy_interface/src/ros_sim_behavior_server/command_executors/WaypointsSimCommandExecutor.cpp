#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToXY.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/planner/commands/Command.h"

#include "ros_sim_behavior_server/command_executors/WaypointsSimCommandExecutor.h"
#include "underwater_autonomy/planner/commands/WaypointsCommand.h"

using namespace underwater_autonomy;

WaypointsSimCommandExecutor::WaypointsSimCommandExecutor(underwater_autonomy::WaypointsCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    CommandExecutor(action),
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0),
    statePropSetup(false),
    poseAtFirstExecuteValid(false)
{
    propStateSub = nh.subscribe("prop_state", 10, &WaypointsSimCommandExecutor::propStateCallback, this);
    poseSub = nh.subscribe("primary_navigation", 1, &WaypointsSimCommandExecutor::navigationFilterCallback, this);
    goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");
}

void WaypointsSimCommandExecutor::execute()
{
    ROS_INFO("ROS: Execute Point Path Command");

    //Check that we have someone listening to us
    goToXYClient.waitForExistence(ros::Duration(10));
    if(!goToXYClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }

    //Wait for the propulsion controller state subscriber to be setup
    waitForPropStateSetup();

    if(!poseAtFirstExecuteValid) {
        poseAtFirstExecute = currentPose;
        poseAtFirstExecuteValid = true;
    }

    //Creates an action goal and sends it to the action server for point path movement
    if(!action.isDone())
    {
        if(sendNextGoToXYGoal())
        {
            action.dispatchDone();
        }
    }
    else
    {
        action.dispatchDone();
        action.complete(action.getLatestTime());
        ROS_INFO("ROS: Point Path Command Completed");
    }

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void WaypointsSimCommandExecutor::monitor()
{
    if(action.doReplan(false, action.getLatestTime() - lastReplanTime, distanceSinceReplan)) 
    {
        replanNextUpdate = true;
    }
}

void WaypointsSimCommandExecutor::stop()
{
    underwater_vehicle_msgs::GoToXY enableMsg;
    enableMsg.request.enable = false;
    if(goToXYClient.exists() &&
       goToXYClient.call(enableMsg))
    {
        if(action.getPointType() == WaypointsCommand::PointType::VEHICLE_RELATIVE) {
            //TODO: Set interrupt point correctly
        }
        else if(action.getPointType() == WaypointsCommand::PointType::WORLD_RELATIVE) {
            Eigen::Vector3d currentPosition = currentPose.getPosition();
            Eigen::Vector3d updatedPosition;

            updatedPosition[0] = currentPosition[0] - poseAtFirstExecute.getPosition()[0];
            updatedPosition[1] = currentPosition[1] - poseAtFirstExecute.getPosition()[1];
            action.setInterruptPoint(updatedPosition);
        } else {
            action.setInterruptPoint(currentPose.getPosition());
        }
        action.stopDone();
    }

    ROS_INFO("ROS: Point Path Command Stopped");
}

bool WaypointsSimCommandExecutor::triggerReplan()
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

void WaypointsSimCommandExecutor::propStateCallback(const underwater_vehicle_msgs::PropulsionControllerState state)
{
    if(!statePropSetup) {
        statePropSetup = true;
        prevXYSeqNum = state.xySeqNum;
    }

    Eigen::Vector3d currentTargetPoint = action.getCurrentTargetPoint();
    if(action.getPointType() == WaypointsCommand::PointType::VEHICLE_RELATIVE) {
        Eigen::Vector3d rpyAngles = poseAtFirstExecute.getOrientation().toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector3d position = poseAtFirstExecute.getPosition();
        double yaw = rpyAngles[2];
        double updatedX = currentTargetPoint[0] * std::cos(yaw) - currentTargetPoint[1] * std::sin(yaw) + position[0];
        double updatedY = currentTargetPoint[0] * std::sin(yaw) + currentTargetPoint[1] * std::cos(yaw) + position[1];

        currentTargetPoint[0] = updatedX;
        currentTargetPoint[1] = updatedY;
    } else if(action.getPointType() == WaypointsCommand::PointType::WORLD_RELATIVE) {
        Eigen::Vector3d position = poseAtFirstExecute.getPosition();
        currentTargetPoint[0] += position[0];
        currentTargetPoint[1] += position[1];
    }


    if(state.xyComplete && 
       doubleEq(currentTargetPoint[0], state.x) &&
       doubleEq(currentTargetPoint[1], state.y) &&
       action.getState() == Command::State::EXECUTING &&
       (prevXYSeqNum != state.xySeqNum))
    {
        prevXYSeqNum = state.xySeqNum;

        action.reachedTargetPoint();
        if(!action.isDone())
        {
            sendNextGoToXYGoal();

            if(action.doReplan(true, action.getLatestTime() - lastReplanTime, distanceSinceReplan)) 
            {
                replanNextUpdate = true;
            }
        }    
        else
        {
            action.complete(action.getLatestTime());
            ROS_INFO("ROS: Point Path Command Complete");
        }
    }
}

void WaypointsSimCommandExecutor::waitForPropStateSetup()
{
    while(!statePropSetup) {
        ros::spinOnce();
    }
}

bool WaypointsSimCommandExecutor::sendNextGoToXYGoal()
{
    Eigen::Vector3d point = action.getCurrentTargetPoint();
    if(action.getPointType() == WaypointsCommand::PointType::VEHICLE_RELATIVE) {
        Eigen::Vector3d rpyAngles = poseAtFirstExecute.getOrientation().toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector3d position = poseAtFirstExecute.getPosition();
        double yaw = rpyAngles[2];
        double updatedX = point[0] * std::cos(yaw) - point[1] * std::sin(yaw) + position[0];
        double updatedY = point[0] * std::sin(yaw) + point[1] * std::cos(yaw) + position[1];

        point[0] = updatedX;
        point[1] = updatedY;
    } else if(action.getPointType() == WaypointsCommand::PointType::WORLD_RELATIVE) {
        Eigen::Vector3d position = poseAtFirstExecute.getPosition();
        point[0] += position[0];
        point[1] += position[1];
    }

    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = point[0];
    goToXYMsg.request.y = point[1];
    goToXYMsg.request.enable = true;

    goToXYMsg.request.xLinearVelocity = action.getTargetHorizontalVelocity();
    goToXYMsg.request.zAngularVelocity = action.getTargetRotationalVelocity();
    
    if(!goToXYClient.call(goToXYMsg)) {
        action.fail(action.getLatestTime());
        return false;
    }

    return true;
}

void WaypointsSimCommandExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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
    zeroedPosition[2] = 0;
    zeroedCurrentPosition[2] = 0;
    distanceSinceReplan += (zeroedPosition - zeroedCurrentPosition).norm();

    currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);

    //Check if out of region
    if((action.getState() == Command::State::DISPATCHING ||
        action.getState() == Command::State::EXECUTING ||
        action.getState() == Command::State::PAUSING ||
        action.getState() == Command::State::COMPLETING) && 
        !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(action.getLatestTime());
    }
}

bool WaypointsSimCommandExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}