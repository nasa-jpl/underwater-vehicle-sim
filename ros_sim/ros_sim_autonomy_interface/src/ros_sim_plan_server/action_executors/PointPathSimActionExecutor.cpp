#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToXY.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "propulsion_controller/PropulsionControllerState.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "ros_sim_plan_server/action_executors/PointPathSimActionExecutor.h"
#include "underwater_autonomy/planner/actions/PointPathAction.h"

using namespace underwater_autonomy;

PointPathSimActionExecutor::PointPathSimActionExecutor(ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0)
{
    velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
    poseSub = nh.subscribe("primary_navigation", 1, &PointPathSimActionExecutor::navigationFilterCallback, this);

    goToXYClient = nh.serviceClient<underwater_vehicle_msgs::GoToXY>("go_to_xy");

    goToXYComplete = nh.subscribe("go_to_xy_complete", 1, &PointPathSimActionExecutor::goToXYCompleteCallback, this);    
}

void PointPathSimActionExecutor::execute(underwater_autonomy::PointPathAction& action)
{
    ROS_INFO("Execute point path action");

    if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
    {
        //Send target velocities command
        geometry_msgs::Twist velMsg;

        velMsg.linear.x = action.getTargetHorizontalVelocity();
        velMsg.linear.y = 0;
        //Calculate the target vertical velocity based on target horizontal velocity and target slope
        velMsg.linear.z = std::numeric_limits<double>::quiet_NaN();

        velMsg.angular.x = 0;
        velMsg.angular.y = 0;
        velMsg.angular.z = action.getTargetRotationalVelocity();
    
        velPub.publish(velMsg);
    }
    else //If the prop module is not known then this cannot be completed
    {
        action.fail(action.getLatestTime());
        return;
    }

    //Check that we have someone listening to us
    ros::WallTime time = ros::WallTime::now();
    while(!goToXYClient.exists() &&
          ros::WallTime::now() - time < ros::WallDuration(5)) {ros::WallDuration(1).sleep();}
    if(!goToXYClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }


    //Creates an action goal and sends it to the action server for point path movement
    if(!action.isDone())
    {
        sendNextGoToXYGoal(action);
        action.dispatchDone();
    }
    else
    {
        action.dispatchDone();
        action.complete(action.getLatestTime());
        ROS_INFO("Point path action completed");
    }

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void PointPathSimActionExecutor::monitor(underwater_autonomy::PointPathAction& action)
{
    Eigen::Vector3d currentTargetPoint = action.getCurrentTargetPoint();

    bool pointReached = false;
    if(gotCompleteCallback && 
       doubleEq(currentTargetPoint[0], completeCallbackX) &&
       doubleEq(currentTargetPoint[1], completeCallbackY) &&
       action.getState() == Action::State::EXECUTING)
    {
        gotCompleteCallback = false;
        action.reachedTargetPoint();
        if(action.isDone())
        {
            action.complete(action.getLatestTime());
            ROS_INFO("Point path action completed");
        }
        else
        {
            sendNextGoToXYGoal(action);
        }
        pointReached = true;
    }
    else if((action.getState() == Action::State::DISPATCHED ||
             action.getState() == Action::State::EXECUTING ||
             action.getState() == Action::State::PAUSING ||
             action.getState() == Action::State::COMPLETING) && 
             !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(action.getLatestTime());
        return;
    }
    else if(action.inStoppingState())
    {
        stop(action);
    }
    
    if(!replanNextUpdate)
    {
        replanNextUpdate = action.doReplan(pointReached,
                                            action.getLatestTime() - lastReplanTime,
                                            distanceSinceReplan);
    }
}

void PointPathSimActionExecutor::stop(underwater_autonomy::PointPathAction& action)
{
    underwater_vehicle_msgs::GoToXY enableMsg;
    enableMsg.request.enable = false;
    if(goToXYClient.exists() &&
       goToXYClient.call(enableMsg))
    {
        action.setInterruptPoint(currentPose.getPosition());
        action.stopDone();
    }

    ROS_INFO("point path action stopped");
}

bool PointPathSimActionExecutor::triggerReplan(underwater_autonomy::PointPathAction& action)
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

void PointPathSimActionExecutor::goToXYCompleteCallback(const underwater_vehicle_msgs::GoToXYComplete complete)
{
    gotCompleteCallback = true;
    completeCallbackX = complete.x;
    completeCallbackY = complete.y;
}

void PointPathSimActionExecutor::sendNextGoToXYGoal(underwater_autonomy::PointPathAction& action)
{
    //Reset the gotCompleteCallback as this might have tripped on previous actions
    gotCompleteCallback = false;

    Eigen::Vector3d point = action.getCurrentTargetPoint();
    underwater_vehicle_msgs::GoToXY goToXYMsg;
    goToXYMsg.request.x = point[0];
    goToXYMsg.request.y = point[1];
    goToXYMsg.request.enable = true;
    
    goToXYClient.call(goToXYMsg);
}

void PointPathSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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
}

bool PointPathSimActionExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}