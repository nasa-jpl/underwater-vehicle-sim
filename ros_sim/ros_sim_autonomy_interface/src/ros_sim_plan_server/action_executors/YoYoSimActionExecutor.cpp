#include "ros_sim_plan_server/action_executors/YoYoSimActionExecutor.h"

#include <vector>
#include <unordered_map>
#include <limits>

#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "underwater_vehicle_msgs/GoToZ.h"

#include "underwater_vehicle_msgs/PropulsionControllerState.h"

#include "underwater_autonomy/planner/actions/Action.h"
#include "underwater_autonomy/planner/actions/YoYoAction.h"

using namespace underwater_autonomy;

YoYoSimActionExecutor::YoYoSimActionExecutor(underwater_autonomy::YoYoAction& action, ros::NodeHandle& nh, VehicleInfo& vehicleInfo) :
    ActionExecutor(action),
    vehicleInfo(vehicleInfo),
    replanNextUpdate(false),
    lastReplanTime(0),
    distanceSinceReplan(0),
    zCompleteState(false)
{
    propStateSub = nh.subscribe("prop_state", 1, &YoYoSimActionExecutor::propStateCallback, this);
    velPub = nh.advertise<geometry_msgs::Twist>("command_target_velocity", 1000, true);
    poseSub = nh.subscribe("primary_navigation", 1, &YoYoSimActionExecutor::navigationFilterCallback, this);
    goToZClient = nh.serviceClient<underwater_vehicle_msgs::GoToZ>("go_to_z");
}

void YoYoSimActionExecutor::execute()
{
    ROS_INFO("Execute yoyo action");

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
        action.fail(action.getLatestTime());
        return;
    }

    waitForPropStateSetup();
    
    //Check that we have someone listening to us
    goToZClient.waitForExistence(ros::Duration(10));
    if(!goToZClient.exists())
    {
        action.fail(action.getLatestTime());
        return;
    }


    //Creates an action goal and sends it to the action server for point path movement
    sendNewGoToZGoal();
    action.dispatchDone();

    lastReplanTime = action.getLatestTime();
    distanceSinceReplan = 0;
}

void YoYoSimActionExecutor::stop()
{
    underwater_vehicle_msgs::GoToZ enableMsg;
    enableMsg.request.enable = false;
    if(goToZClient.exists() &&
       goToZClient.call(enableMsg))
    {
        action.stopDone();
    }
}

bool YoYoSimActionExecutor::triggerReplan()
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

void YoYoSimActionExecutor::propStateCallback(const underwater_vehicle_msgs::PropulsionControllerState state)
{    
    if(!statePropSetup) {
        statePropSetup = true;
        prevZSeqNum = state.zSeqNum;
    }

    if(state.zComplete && (prevZSeqNum != state.zSeqNum))
    {
        zCompleteState = true;
        zState = state.z;
        holdDepthState = state.holdDepth;
        zSeqNumState = state.zSeqNum;
    }
}

void YoYoSimActionExecutor::waitForPropStateSetup()
{
    while(!statePropSetup) {
        ros::spinOnce();
    }
}

void YoYoSimActionExecutor::monitor()
{
    if((action.getState() == Action::State::DISPATCHED ||
        action.getState() == Action::State::EXECUTING ||
        action.getState() == Action::State::PAUSING ||
        action.getState() == Action::State::COMPLETING) &&
       !action.inOperationRegion(currentPose.getPosition()))
    {
        action.fail(action.getLatestTime());
    }  

    if(action.getState() == Action::State::EXECUTING)
    {
        double targetDepth = 0;
        if(action.getGoingUp())
        {
            targetDepth = action.getUpperDepth();
        }
        else
        {
            targetDepth = action.getLowerDepth();
        }

        if(zCompleteState && 
           doubleEq(targetDepth, zState) &&
           !holdDepthState)
        {
            action.setGoingUp(!action.getGoingUp());
            sendNewGoToZGoal();

            if(action.getState() == Action::State::EXECUTING && !replanNextUpdate)
            {
                replanNextUpdate = action.doReplan(true, action.getLatestTime() - lastReplanTime,
                                                    distanceSinceReplan);
            }
        }
        
        if(action.getYoYoTime() >= 0 && action.getTimeRunning() >= action.getYoYoTime())
        {
            action.complete(action.getLatestTime());
        }


    }
    else if(action.inStoppingState())
    {
        stop();
    }

    if(action.getState() == Action::State::EXECUTING && !replanNextUpdate)
    {
        replanNextUpdate = action.doReplan(false, action.getLatestTime() - lastReplanTime,
                                            distanceSinceReplan);
    }
}

void YoYoSimActionExecutor::sendNewGoToZGoal()
{

    //Update the previous sequence number to the current one so we can use it again
    //for the next command
    prevZSeqNum = zSeqNumState;

    //Reset the xyCompleteState as this might have tripped on previous actions
    zCompleteState = false;

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

    goToZClient.call(goToZMsg);
}

void YoYoSimActionExecutor::navigationFilterCallback(const nav_msgs::Odometry odo)
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

bool YoYoSimActionExecutor::doubleEq(double d1, double d2)
{
    return abs(d1 - d2) < 0.001;
}