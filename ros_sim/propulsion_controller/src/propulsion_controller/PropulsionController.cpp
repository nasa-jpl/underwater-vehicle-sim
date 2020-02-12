#include "propulsion_controller/PropulsionController.h"
#include "std_msgs/Bool.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"


PropulsionController::PropulsionController(VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController) :
    PropulsionController(ros::NodeHandle(), info, std::move(logicController))
{}

PropulsionController::PropulsionController(ros::NodeHandle nh, VehicleInfo& info, std::unique_ptr<PropulsionLogicInterface> logicController) :
    nh(nh),
    info(info),
    logicController(std::move(logicController)),
    goToZEnable(false),
    goToZHoldDepth(false),
    goToXYEnable(false),
    followHeadingEnable(false)

{
    std::vector<std::string> dataModuleNames = info.getModuleNamesOfType("DataBroadcaster");
    if(dataModuleNames.size() > 0)
    {
        //default to using first module of type DataBroadcaster if more than 1 exists
        dataSub = nh.subscribe(dataModuleNames[0] + "/data", 1, &PropulsionController::getVehicleData, this);
    }

    velSub = nh.subscribe("command_target_velocity", 10, &PropulsionController::getTargetVelocityCommand, this);
    poseSub = nh.subscribe("primary_navigation", 1, &PropulsionController::navigationFilterCallback, this);

    //Go To Z Topics
    goToZSub = nh.subscribe("go_to_z", 10, &PropulsionController::goToZCallback, this);
    goToZEnableService = nh.advertiseService("go_to_z_enable", &PropulsionController::goToZEnableCallback, this);
    goToZComplete = nh.advertise<underwater_vehicle_msgs::GoToZComplete>("go_to_z_complete", 1000);

    //Go To XY Topics
    goToXYSub = nh.subscribe("go_to_xy", 10, &PropulsionController::goToXYCallback, this);
    goToXYEnableService = nh.advertiseService("go_to_xy_enable", &PropulsionController::goToXYEnableCallback, this);
    goToXYComplete = nh.advertise<underwater_vehicle_msgs::GoToXYComplete>("go_to_xy_complete", 1000);

    //Follow Heading Topics
    followHeadingSub = nh.subscribe("follow_heading", 10, &PropulsionController::followHeadingCallback, this);
    followHeadingEnableService = nh.advertiseService("follow_heading_enable", &PropulsionController::followHeadingEnableCallback, this);

    stateService = nh.advertiseService("get_state", &PropulsionController::getState, this);
}

void PropulsionController::getTargetVelocityCommand(const geometry_msgs::Twist vel)
{
    logicController->setTargetVelocity(vel);
}

void PropulsionController::getVehicleData(const underwater_vehicle_msgs::VehicleData data)
{
    logicController->processNewData(data);
}

void PropulsionController::update(void)
{
    //Only allow one type of XY commanding at a time.
    if(goToXYEnable)
    {
        goToXYUpdate();
    }
    else if(followHeadingEnable)
    {
        followHeadingUpdate();
    }

    if(goToZEnable)
    {
        goToZUpdate();
    }
    else
    {
        logicController->avoidSeafloor(currentPose);
    }
}

void PropulsionController::goToXYCallback(const underwater_vehicle_msgs::GoToXY parameters)
{
    ROS_INFO("GO TO XY: x=%f, y=%f", parameters.x, parameters.y);
    logicController->setTargetXY(parameters.x, parameters.y);
    goToXYEnable = parameters.enable;
    if(goToXYEnable)
    {
        followHeadingEnable = false;
    }
}

bool PropulsionController::goToXYEnableCallback(propulsion_controller::PropulsionControllerEnable::Request  &req,
                                                propulsion_controller::PropulsionControllerEnable::Response &res)
{
    ROS_INFO("GO TO XY ENABLE: %d", req.enable);
    if(req.enable)
    {
        goToXYEnable = true;
        followHeadingEnable = false;
    }
    else
    {
        logicController->stopXY();
        goToXYEnable = false;
    }
    return true;
}

void PropulsionController::goToXYUpdate(void)
{    
    if(logicController->isAtXY(currentPose))
    {
        logicController->stopXY();
        goToXYEnable = false;

        underwater_vehicle_msgs::GoToXYComplete completeMsg;
        completeMsg.x = logicController->getTargetX();
        completeMsg.y = logicController->getTargetY();
        goToXYComplete.publish(completeMsg);
        ROS_INFO("GO TO XY COMPLETE: x=%f, y=%f", completeMsg.x, completeMsg.y);
        ROS_DEBUG("GoToXY goal completed");
    }
    else
    {
        logicController->goToXY(currentPose);
    }
}

void PropulsionController::followHeadingCallback(const underwater_vehicle_msgs::FollowHeading parameters)
{
    followHeadingEnable = parameters.enable;
    if(followHeadingEnable)
    {
        goToXYEnable = false;
    }
    logicController->setFollowHeading(parameters.heading);
}

bool PropulsionController::followHeadingEnableCallback(propulsion_controller::PropulsionControllerEnable::Request  &req,
                                                propulsion_controller::PropulsionControllerEnable::Response &res)
{
    if(req.enable)
    {
        followHeadingEnable = true;
        goToXYEnable = false;
    }
    else
    {
        logicController->stopXY();
        followHeadingEnable = false;
    }
    return true;
}

void PropulsionController::followHeadingUpdate(void)
{    
    logicController->followHeading(currentPose);
}

void PropulsionController::goToZCallback(const underwater_vehicle_msgs::GoToZ parameters)
{
    goToZHoldDepth = parameters.holdDepth;
    goToZEnable = parameters.enable;
    logicController->setTargetZ(parameters.depth);
}

bool PropulsionController::goToZEnableCallback(propulsion_controller::PropulsionControllerEnable::Request  &req,
                                                propulsion_controller::PropulsionControllerEnable::Response &res)
{
    if(req.enable)
    {
        goToZEnable = true;
    }
    else
    {
        logicController->stopZ();
        goToZEnable = false;
    }
    return true;
}

void PropulsionController::goToZUpdate(void)
{
    if(logicController->isAtZ(currentPose) && !goToZHoldDepth)
    {
        logicController->stopZ();
        goToZEnable = false;

        underwater_vehicle_msgs::GoToZComplete completeMsg;
        completeMsg.depth = logicController->getTargetZ();
        completeMsg.holdDepth = goToZHoldDepth;
        goToZComplete.publish(completeMsg);
        ROS_DEBUG("GoToZ goal completed");
    }
    else
    {
        logicController->goToZ(currentPose);
    }
}

void PropulsionController::navigationFilterCallback(const nav_msgs::Odometry odo)
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

    currentPose.setPosition(position);
    currentPose.setOrientation(orientation);
    currentPose.setPoseCovariance(poseCovariance);
    
    currentPose.setLinearVelocity(linearVelocity);
    currentPose.setAngularVelocity(angularVelocity);
    currentPose.setTwistCovariance(twistCovariance);

    update();
}

bool PropulsionController::getState(propulsion_controller::PropulsionControllerState::Request  &req,
                                    propulsion_controller::PropulsionControllerState::Response &res)
{
    res.xyEnabled = goToXYEnable;
    res.zEnabled = goToZEnable;
    res.holdDepth = goToZHoldDepth;
    res.followHeadingEnabled = followHeadingEnable;
    return true;
}