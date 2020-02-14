#include "ros_sim_plan_server/action_executors/HoldDepthSimActionExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToZ.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "propulsion_controller/PropulsionControllerState.h"
#include "propulsion_controller/PropulsionControllerEnable.h"

#include "underwater_autonomy/planner/actions/Action.h"

#include "underwater_autonomy/planner/actions/HoldDepthAction.h"

using namespace underwater_autonomy;

HoldDepthSimActionExecutor::HoldDepthSimActionExecutor(ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplan(ros::Time::now()),
    distanceSinceReplan(0),
    gotCompleteCallback(false)
{
    velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
    poseSub = nh.subscribe("primary_navigation", 1, &HoldDepthSimActionExecutor::navigationFilterCallback, this);

    goToZPub = nh.advertise<underwater_vehicle_msgs::GoToZ>("go_to_z", 1000);
    goToZEnableClient = nh.serviceClient<propulsion_controller::PropulsionControllerEnable>("go_to_z_enable");
    goToZComplete = nh.subscribe("go_to_z_complete", 1, &HoldDepthSimActionExecutor::goToZCompleteCallback, this);    
}

void HoldDepthSimActionExecutor::execute(underwater_autonomy::HoldDepthAction& action)
{
    ROS_INFO("Execute hold depth action");

    if(vehicleInfo.getPropModuleType() == "FourDOFPropulsion")
    {
        //Send target velocities command
        geometry_msgs::Twist velMsg;

        //xy is set to nan as we do not want to modify it
        velMsg.linear.x = std::numeric_limits<double>::quiet_NaN();
        velMsg.linear.y = std::numeric_limits<double>::quiet_NaN();

        velMsg.linear.z = action.getTargetVerticalVelocity();

        velMsg.angular.x = std::numeric_limits<double>::quiet_NaN();
        velMsg.angular.y = std::numeric_limits<double>::quiet_NaN();
        velMsg.angular.z = std::numeric_limits<double>::quiet_NaN();
    
        velPub.publish(velMsg);
    }
    else //If the prop module is not known then this cannot be completed
    {
        action.fail(ros::Time::now().toSec());
    }

    //Check that we have someone listening to us
    ros::WallTime time = ros::WallTime::now();
    while(goToZPub.getNumSubscribers() == 0 &&
          ros::WallTime::now() - time < ros::WallDuration(5)) {ros::WallDuration(1).sleep();}
    if(goToZPub.getNumSubscribers() == 0)
    {
        action.fail(ros::Time::now().toSec());
    }

    //Send message to Go To Z Controller
    //Reset complete callback
    gotCompleteCallback = false;
    underwater_vehicle_msgs::GoToZ goToZMsg;
    goToZMsg.depth = action.getDepth();
    goToZMsg.enable = true;
    goToZMsg.holdDepth = true;
    goToZPub.publish(goToZMsg);
    action.dispatchDone();
    
    lastReplan = ros::Time::now();
    distanceSinceReplan = 0;
}

void HoldDepthSimActionExecutor::stop(underwater_autonomy::HoldDepthAction& action)
{
    propulsion_controller::PropulsionControllerEnable enableMsg;
    enableMsg.request.enable = false;
    if(goToZEnableClient.exists() &&
       goToZEnableClient.call(enableMsg))
    {
        action.stopDone();
    }
}

bool HoldDepthSimActionExecutor::triggerReplan(underwater_autonomy::HoldDepthAction& action)
{
    if(replanNextUpdate)
    {        
        replanNextUpdate = false;
        lastReplan = ros::Time::now();
        distanceSinceReplan = 0;
        return true;
    }

    return false;
}

void HoldDepthSimActionExecutor::goToZCompleteCallback(const underwater_vehicle_msgs::GoToZComplete complete)
{
    gotCompleteCallback = true;
    completeCallbackZ = complete.depth;
    completeCallbackHoldDepth = complete.holdDepth;
}

void HoldDepthSimActionExecutor::monitor(underwater_autonomy::HoldDepthAction& action)
{
    if((action.getState() == Action::State::DISPATCHED ||
        action.getState() == Action::State::EXECUTING ||
        action.getState() == Action::State::PAUSING ||
        action.getState() == Action::State::COMPLETING) &&
       !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(ros::Time::now().toSec());
    }

    if(action.getState() == Action::State::EXECUTING)
    {
        if(gotCompleteCallback && 
           doubleEq(action.getDepth(), completeCallbackZ) &&
           completeCallbackHoldDepth)
        {
            gotCompleteCallback = false;
            action.complete(ros::Time::now().toSec());
        }
        else if(action.getHoldDepthTime() >= 0 && action.getTimeRunning() >= action.getHoldDepthTime())
        {
            action.complete(ros::Time::now().toSec());
        }
    }
    else if(action.inStoppingState())
    {
        stop(action);
    }

    if(action.getState() == Action::State::EXECUTING && !replanNextUpdate)
    {
        replanNextUpdate = action.doReplan((ros::Time::now() - lastReplan).toSec(),
                                             distanceSinceReplan);
    }
}

void HoldDepthSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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
}

bool HoldDepthSimActionExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}