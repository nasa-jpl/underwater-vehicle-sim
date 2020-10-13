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
    zSeqNum(0),
    goToZComplete(false),
    goToZHoldDepth(false),
    goToXYEnable(false),
    xySeqNum(0),
    goToXYComplete(false),
    followHeadingEnable(false)
{
    std::vector<std::string> dataModuleNames = info.getModuleNamesOfType("DataBroadcaster");
    if(dataModuleNames.size() > 0)
    {
        //default to using first module of type DataBroadcaster if more than 1 exists
        dataSub = nh.subscribe(dataModuleNames[0] + "/data", 10, &PropulsionController::getVehicleData, this);
    }

    poseSub = nh.subscribe("primary_navigation", 1, &PropulsionController::navigationFilterCallback, this);

    //Go To Z Topics
    goToZService = nh.advertiseService("go_to_z", &PropulsionController::goToZCallback, this);

    //Go To XY Topics
    goToXYService = nh.advertiseService("go_to_xy", &PropulsionController::goToXYCallback, this);

    //Follow Heading Topics
    followHeadingService = nh.advertiseService("follow_heading", &PropulsionController::followHeadingCallback, this);

    statePub = nh.advertise<underwater_vehicle_msgs::PropulsionControllerState>("prop_state", 1000);
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
    } else {
        logicController->stopXY();
    }

    if(goToZEnable)
    {
        goToZUpdate();
    }
    else
    {
        logicController->avoidSeafloor(currentPose);
    }

    publishState();
}

bool PropulsionController::goToXYCallback(underwater_vehicle_msgs::GoToXY::Request  &req,
                                          underwater_vehicle_msgs::GoToXY::Response &res)
{
    //Reset the complete flag
    goToXYComplete = false;

    if(req.enable)
    {
        logicController->setTargetXY(req.x, req.y);
        logicController->setVelocityXY(req.xLinearVelocity, 0, req.zAngularVelocity);
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
        goToXYComplete = true;
        xySeqNum++;

        ROS_DEBUG("GoToXY goal completed");
    }
    else
    {
        logicController->goToXY(currentPose);
    }
}

bool PropulsionController::followHeadingCallback(underwater_vehicle_msgs::FollowHeading::Request  &req,
                                                underwater_vehicle_msgs::FollowHeading::Response &res)
{
    //Reset the complete flag when follow heading as well
    goToXYComplete = false;

    if(req.enable)
    {
        followHeadingEnable = true;
        goToXYEnable = false;
        logicController->setFollowHeading(req.heading);
        logicController->setVelocityXY(req.xLinearVelocity, 0, req.zAngularVelocity);
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

bool PropulsionController::goToZCallback(underwater_vehicle_msgs::GoToZ::Request  &req,
                                         underwater_vehicle_msgs::GoToZ::Response &res)
{
    //Reset the complete flag
    goToZComplete = false;

    if(req.enable)
    {
        goToZEnable = true;
        goToZHoldDepth = req.holdDepth;
        logicController->setTargetZ(req.depth);
        logicController->setVelocityZ(req.zLinearVelocity);
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
        goToZComplete = true;
        zSeqNum++;

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

void PropulsionController::publishState() 
{
    underwater_vehicle_msgs::PropulsionControllerState stateMsg;

    stateMsg.header.stamp = ros::Time::now();

    stateMsg.xyEnable = goToXYEnable;
    stateMsg.xyComplete = goToXYComplete;
    stateMsg.xySeqNum = xySeqNum;
    stateMsg.x = logicController->getTargetX();
    stateMsg.y = logicController->getTargetY();

    stateMsg.zEnable = goToZEnable;
    stateMsg.zComplete = goToZComplete;
    stateMsg.zSeqNum = zSeqNum;
    stateMsg.z = logicController->getTargetZ();
    stateMsg.holdDepth = goToZHoldDepth;

    statePub.publish(stateMsg);
}