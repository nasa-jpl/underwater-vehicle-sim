#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/FollowHeading.h"

#include "propulsion_controller/PropulsionControllerState.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "ros_sim_plan_server/action_executors/FollowHeadingSimActionExecutor.h"

using namespace underwater_autonomy;

FollowHeadingSimActionExecutor::FollowHeadingSimActionExecutor(underwater_autonomy::FollowHeadingAction& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    ActionExecutor(action),
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0)
{
    poseSub = nh.subscribe("primary_navigation", 1, &FollowHeadingSimActionExecutor::navigationFilterCallback, this);

    followHeadingClient = nh.serviceClient<underwater_vehicle_msgs::FollowHeading>("follow_heading");
}

void FollowHeadingSimActionExecutor::execute()
{
    ROS_INFO("Execute follow heading action");

    //Check that we have someone listening to us
    followHeadingClient.waitForExistence(ros::Duration(10));
    if(!followHeadingClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }

    //Send message to Follow Heading Controller
    //Reset complete callback
    underwater_vehicle_msgs::FollowHeading followHeadingMsg;
    followHeadingMsg.request.heading = action.getHeading();
    followHeadingMsg.request.enable = true;
    followHeadingMsg.request.xLinearVelocity = action.getTargetHorizontalVelocity();
    followHeadingMsg.request.zAngularVelocity = action.getTargetRotationalVelocity();

    if(followHeadingClient.call(followHeadingMsg)) 
    {
        action.dispatchDone();
    } 
    else 
    {
        action.fail(action.getLatestTime());
        return;
    }

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void FollowHeadingSimActionExecutor::stop()
{
    underwater_vehicle_msgs::FollowHeading enableMsg;
    enableMsg.request.enable = false;
    if(followHeadingClient.exists() &&
       followHeadingClient.call(enableMsg))
    {
        action.stopDone();
    }
    ROS_INFO("Stop follow heading action");
}

bool FollowHeadingSimActionExecutor::triggerReplan()
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

void FollowHeadingSimActionExecutor::monitor()
{
    if(action.getFollowHeadingTime() >= 0 && 
       action.getTimeRunning() >= action.getFollowHeadingTime() &&
       action.getState() == Action::State::EXECUTING)
    {
        action.complete(action.getLatestTime());
        ROS_INFO("Complete follow heading action");
    }
    else if(action.doReplan(action.getLatestTime() - lastReplanTime, distanceSinceReplan))
    {
        replanNextUpdate = true;
    }
}

void FollowHeadingSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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

    if((action.getState() == Action::State::DISPATCHED ||
        action.getState() == Action::State::EXECUTING ||
        action.getState() == Action::State::PAUSING ||
        action.getState() == Action::State::COMPLETING) &&
        !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(action.getLatestTime());
    }
}