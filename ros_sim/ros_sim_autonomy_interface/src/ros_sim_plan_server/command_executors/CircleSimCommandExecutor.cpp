#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToXY.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/planner/commands/Command.h"

#include "ros_sim_plan_server/command_executors/CircleSimCommandExecutor.h"
#include "underwater_autonomy/planner/commands/CircleCommand.h"

using namespace underwater_autonomy;

CircleSimCommandExecutor::CircleSimCommandExecutor(underwater_autonomy::CircleCommand& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    CommandExecutor(action),
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0),
    statePropSetup(false)
{
    propStateSub = nh.subscribe("prop_state", 10, &CircleSimCommandExecutor::propStateCallback, this);
    poseSub = nh.subscribe("primary_navigation", 1, &CircleSimCommandExecutor::navigationFilterCallback, this);
    goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");
}

void CircleSimCommandExecutor::execute()
{
    ROS_INFO("ROS: Execute Circle Command");

    //Check that we have someone listening to us
    goToXYClient.waitForExistence(ros::Duration(10));
    if(!goToXYClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }

    //Wait for the propulsion controller state subscriber to be setup
    waitForPropStateSetup();

    createCirclePoints();
    currentCirclePoint = 0;

    //Creates an action goal and sends it to the action server for point path movement
    if(sendNextGoToXYGoal())
    {
        action.dispatchDone();
    }

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void CircleSimCommandExecutor::monitor()
{
    if(action.getState() == Command::State::EXECUTING &&
       action.getCircleTime() >= 0 &&
       action.getTimeRunning() >= action.getCircleTime())
    {
        action.complete(action.getLatestTime());
        ROS_INFO("ROS: Complete Circle Command");
    }

    if(action.doReplan(action.getLatestTime() - lastReplanTime, distanceSinceReplan)) 
    {
        replanNextUpdate = true;
    }
}

void CircleSimCommandExecutor::stop()
{
    underwater_vehicle_msgs::GoToXY enableMsg;
    enableMsg.request.enable = false;
    if(goToXYClient.exists() &&
       goToXYClient.call(enableMsg))
    {
        action.stopDone();
    }

    ROS_INFO("ROS: Circle Command Stopped");
}

bool CircleSimCommandExecutor::triggerReplan()
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

void CircleSimCommandExecutor::propStateCallback(const underwater_vehicle_msgs::PropulsionControllerState state)
{
    if(!statePropSetup) {
        statePropSetup = true;
        prevXYSeqNum = state.xySeqNum;
    }

    if(circlePoints.size() == 0) {
        return;
    }

    Eigen::Vector2d currentTargetPoint = circlePoints[currentCirclePoint];

    if(state.xyComplete && 
       doubleEq(currentTargetPoint[0], state.x) &&
       doubleEq(currentTargetPoint[1], state.y) &&
       action.getState() == Command::State::EXECUTING &&
       (prevXYSeqNum != state.xySeqNum))
    {
        prevXYSeqNum = state.xySeqNum;

        currentCirclePoint++;
        if(currentCirclePoint >= circlePoints.size()) {
            currentCirclePoint = 0;
        }
        sendNextGoToXYGoal();
    }
}

void CircleSimCommandExecutor::waitForPropStateSetup()
{
    while(!statePropSetup) {
        ros::spinOnce();
    }
}

bool CircleSimCommandExecutor::sendNextGoToXYGoal()
{
    if(circlePoints.size() == 0) {
        return false;
    }
    Eigen::Vector2d point = circlePoints[currentCirclePoint];

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

void CircleSimCommandExecutor::createCirclePoints() {
    const double pi = 3.14159265358979323846;

    circlePoints.clear();

    Eigen::Vector3d position = currentPose.getPosition();
    double angleInterval = pi / 4;

    for(uint i = 0; i < 8; i++) {
        double x = position[0] + cos(angleInterval * i) * action.getRadius();
        double y = position[1] + sin(angleInterval * i) * action.getRadius();
        circlePoints.emplace_back(x, y);
    }
}

void CircleSimCommandExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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

bool CircleSimCommandExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}