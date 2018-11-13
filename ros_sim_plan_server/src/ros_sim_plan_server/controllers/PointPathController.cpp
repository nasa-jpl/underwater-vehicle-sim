#include "ros_sim_plan_server/controllers/PointPathController.h"

#include "data_server/GetPlumeData.h"

PointPathController::PointPathController(ros::NodeHandle nh, 
                                         VehicleInfo vehicleInfo) :
    nh(nh),
    goToXYClient(nh, "vehicle_controller/" + vehicleInfo.getName() + "/go_to_xy", false),
    goToZClient(nh, "vehicle_controller/" + vehicleInfo.getName() + "/go_to_z", false),
    pointPathServer(nh, "planner/" + vehicleInfo.getName() + "/point_path", false),
    vehicleInfo(vehicleInfo)
{
    std::vector<std::string> data = vehicleInfo.getModuleNamesOfType("DataBroadcaster");
    if(data.size() > 0)
    {
        dataSub = nh.subscribe("vehicles/" + vehicleInfo.getName() + "/" + data[0] + "/data", 1, &PointPathController::getVehicleData, this);
    }

    //Init action server
    pointPathServer.registerGoalCallback(boost::bind(&PointPathController::goalCB, this));
    pointPathServer.registerPreemptCallback(boost::bind(&PointPathController::preemptCB, this));
    pointPathServer.start();
}

void PointPathController::getVehicleData(const underwater_vehicle_msgs::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
    latestVehicleDepth = data.h;
}

void PointPathController::sendAllGoals(void)
{
    xyDone = false;
    sendXYGoal(pathPoints[currentPoint].getX(), pathPoints[currentPoint].getY());

    ROS_INFO("send goals: %d %f %f", currentPoint, pathPoints[currentPoint].getX(), pathPoints[currentPoint].getY());   
    if(!yoyo)
    {
        zDone = false;
        sendZGoal(pathPoints[currentPoint].getZ());
    }
}

void PointPathController::pointPathUpdate(void)
{
    if(xyDone && zDone)
    {
        ROS_INFO("Point path go to next point");
        currentPoint++;
        if(currentPoint >= pathPoints.size())
        {
            ROS_INFO("Point path server set to succeeded");
            ros_sim_plan_server::PointPathRosResult result;
            result.totalPoints = currentPoint;
            pointPathServer.setSucceeded(result);
            return;
        }

        sendAllGoals();
    }
     sendFeedback();
}

void PointPathController::yoyoUpdate(void)
{

    goingUp = !goingUp;
    double targetZ = goingUp ? upperDepth : lowerDepth;
    ROS_INFO("YoYo Update - goingUp: %d; targetZ: %f", goingUp, targetZ);
    sendZGoal(targetZ);

    sendFeedback();
}

void PointPathController::sendFeedback(void)
{
    ros_sim_plan_server::PointPathRosFeedback feedback;
    feedback.currentPoint = currentPoint;
    feedback.goingUp = goingUp;
    pointPathServer.publishFeedback(feedback);
}

void PointPathController::goalCB(void)
{  
    ROS_INFO("Point path server accept new goal");
    ros_sim_plan_server::PointPathRosGoalConstPtr pointPathGoal = pointPathServer.acceptNewGoal();
    currentPoint = 0;
    goingUp = true;
    xyDone = false;
    zDone = pointPathGoal->yoyo; //z is always "done" if yoyoing
    pathPoints.clear();
    for(auto point : pointPathGoal->points)
    {
        pathPoints.emplace_back(point.x, point.y, point.z);
    }
    yoyo = pointPathGoal->yoyo;
    upperDepth = pointPathGoal->upperDepth;
    lowerDepth = pointPathGoal->lowerDepth;

    sendAllGoals();
    if(yoyo)
    {
        yoyoUpdate();
    }
}

void PointPathController::preemptCB(void)
{
    goToXYClient.cancelAllGoals();
    goToZClient.cancelAllGoals();
    pointPathServer.setPreempted();
}
    

void PointPathController::goToXYActive(void) {}
void PointPathController::goToXYFeedback(const vehicle_auto_control::GoToXYRosFeedbackConstPtr& feedback) 
{
    sendFeedback();
}

void PointPathController::goToXYDone(const actionlib::SimpleClientGoalState& state,
                       const vehicle_auto_control::GoToXYRosResultConstPtr& result)
{
    ROS_INFO("GoToXY goal sent by point path controller is done");
    xyDone = true;
    if(pointPathServer.isActive())
    {
        pointPathUpdate(); 
    }
}

void PointPathController::sendXYGoal(const double x, const double y)
{
    //Creates an action goal and sends it to the action server for xy movement
    vehicle_auto_control::GoToXYRosGoal xyGoal = vehicle_auto_control::GoToXYRosGoal();

    xyGoal.x = x;
    xyGoal.y = y;

    ROS_INFO("Waiting for GoToXY server");
    while(!goToXYClient.waitForServer(ros::Duration(5.0)))
    {
        ros::spinOnce();
    }
    goToXYClient.sendGoal(xyGoal,
        boost::bind(&PointPathController::goToXYDone, this, _1, _2),
        boost::bind(&PointPathController::goToXYActive, this),
        boost::bind(&PointPathController::goToXYFeedback, this, _1));
    ROS_INFO("Send goal to GoToXY server from point path controller - x:%f y:%f", x, y);
}

void PointPathController::goToZActive(void) {}
void PointPathController::goToZFeedback(const vehicle_auto_control::GoToZRosFeedbackConstPtr& feedback) 
{
    if(!goingUp && fabs(latestSonarDepth - 10.0) <= 5.0)
    {
        if(yoyo)
        {
            yoyoUpdate();
        }
        else if(xyDone) //Stop if at point in xy direction and at the seafloor
        {
            zDone = true;
            pointPathUpdate();
        }
    }

    sendFeedback();
}

void PointPathController::goToZDone(const actionlib::SimpleClientGoalState& state,
                       const vehicle_auto_control::GoToZRosResultConstPtr& result)
{
    ROS_INFO("GoToZ goal sent by point path controller is done");
    if(pointPathServer.isActive())
    {
        if(yoyo)
        {
            yoyoUpdate();
        }
        else
        {
            
            zDone = true;
            pointPathUpdate();
        } 
    }
}

void PointPathController::sendZGoal(const double z)
{
    //Creates an action goal and sends it to the action server for z movement
    vehicle_auto_control::GoToZRosGoal zGoal = vehicle_auto_control::GoToZRosGoal();

    zGoal.z = z;

    ROS_INFO("Waiting for GoToZ server");
    while(!goToZClient.waitForServer(ros::Duration(5.0)))
    {
        ros::spinOnce();
    }
    goToZClient.sendGoal(zGoal,
        boost::bind(&PointPathController::goToZDone, this, _1, _2),
        boost::bind(&PointPathController::goToZActive, this),
        boost::bind(&PointPathController::goToZFeedback, this, _1));
    ROS_INFO("Send GoToZ goal from point path controller - z:%f", z);
}