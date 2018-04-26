#include <math.h>
#include <algorithm>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "actionlib/server/simple_action_server.h"

#include "geometry_msgs/PointStamped.h"

#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/FourDOFPropulsionController.h"

#include "vehicle_auto_control/DynamicLawnmowerAction.h"
#include "vehicle_auto_control/PointPathAction.h"

#include "underwater_vehicle_sim/VehicleData.h"

#include "data_server/GetPlumeData.h"

FourDOFPropulsionController::FourDOFPropulsionController(ros::NodeHandle controlNode, ros::NodeHandle vehicleNode, std::string propModuleName, std::string dataModuleName, std::string vehicleName, float loopHertz) :
    PropulsionController(controlNode, vehicleNode, vehicleName, loopHertz),
    targetHorzVelocity(0),
    targetRotVelocity(0),
    targetVertVelocity(0),
    lateralError(5.0),
    verticalError(1.0),
    latestSonarDepth(1000),
    latestVehicleDepth(0),
    minSeafloorDistance(10.0),
    pointPathServer(controlNode, "point_path", boost::bind(&FourDOFPropulsionController::executePointPath, this, _1, &pointPathServer), false),
    dynamicLawnmowerServer(controlNode, "dynamic_lawnmower", boost::bind(&FourDOFPropulsionController::executeDynamicLawnmower, this, _1, &dynamicLawnmowerServer), false),
    plumeClient(controlNode.serviceClient<data_server::GetPlumeData>("data_server/get_plume"))
{
    velocityPub = vehicleNode.advertise<geometry_msgs::Twist>(propModuleName + "/command_velocity", 1000);

    if(dataModuleName != "")
    {
        hasVehicleData = true;
        dataSub = vehicleNode.subscribe(dataModuleName + "/data", 1, &FourDOFPropulsionController::getVehicleData, this);
    }

    velocitySub = controlNode.subscribe("command_target_velocity", 1, &FourDOFPropulsionController::getTargetVelocityCommand, this);
    pointPathServer.start();
    dynamicLawnmowerServer.start();
}

void FourDOFPropulsionController::getTargetVelocityCommand(const vehicle_auto_control::Velocity vel)
{
    targetHorzVelocity = fabs(vel.horizontalVelocity);
    targetRotVelocity = fabs(vel.rotationalVelocity);
    targetVertVelocity = fabs(vel.verticalVelocity);
}

void FourDOFPropulsionController::getVehicleData(const underwater_vehicle_sim::VehicleData data)
{
    latestSonarDepth = data.sonarDepth;
    latestVehicleDepth = data.h;
}

void FourDOFPropulsionController::executePointPath(const vehicle_auto_control::PointPathGoalConstPtr& goal, 
                                                   actionlib::SimpleActionServer<vehicle_auto_control::PointPathAction>* as)
{
    //Feedback and Results for the action
    vehicle_auto_control::PointPathFeedback feedback;
    vehicle_auto_control::PointPathResult result;

    //Rate at which to run the control loop
    ros::Rate r(loopHertz);

    std::vector<tf::Vector3> pathPoints;
    for(auto point: goal->points)
    {
        pathPoints.emplace_back(point.x, point.y, point.z);
    }

    unsigned int currentPoint = 0;
    bool goingUp = true;

    while(currentPoint < pathPoints.size() && ros::ok())
    {

        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/world", "/" + vehicleName,
                                      ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/world", "/" + vehicleName,  
                                     ros::Time(0), transform);

            if(currentPoint < pathPoints.size())
            {
                if(isAtPoint(transform, pathPoints[currentPoint], !goal->yoyo))
                {
                    currentPoint++;
                }
            }
                        
            feedback.currentPoint = currentPoint;
            feedback.goingUp = goingUp;
            as->publishFeedback(feedback);
            
            if(as->isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Auto Controller: Point Path Action Preempted");

                //Stop vehicle
                sendVelocityCommand(0,0,0,0);
                as->setPreempted();
                break;
            }

            if(goal->yoyo)
            {
                if(transform.getOrigin().getZ() + verticalError > goal->upperDepth || transform.getOrigin().getZ() + verticalError >= 0)
                {
                    goingUp = false;
                }
                else if(transform.getOrigin().getZ() - verticalError < goal->lowerDepth || fabs(latestSonarDepth - minSeafloorDistance) <= verticalError)
                {
                    goingUp = true;
                }
            }
            
            if(currentPoint < pathPoints.size())
            {
                double targetHeight = 0;
                if(goal->yoyo)
                {
                    targetHeight = goingUp ? goal->upperDepth : goal->lowerDepth;
                }
                else
                {
                    targetHeight = pathPoints[currentPoint].getZ();
                }

                goToPoint(transform, pathPoints[currentPoint], targetHeight);
            }
            else
            {
                //Stop the vehicle
                sendVelocityCommand(0, 0, 0, 0);
            }
            
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        r.sleep();
    }

    if(currentPoint == pathPoints.size())
    {
        ROS_INFO("Auto Controller: Point Path Action Done, Succeeded");
        
        //Stop vehicle
        sendVelocityCommand(0,0,0,0);
        result.totalPoints = currentPoint;
        as->setSucceeded(result);
    }
    
}

void FourDOFPropulsionController::executeDynamicLawnmower(const vehicle_auto_control::DynamicLawnmowerGoalConstPtr& goal, 
                                                          actionlib::SimpleActionServer<vehicle_auto_control::DynamicLawnmowerAction>* as)
{
    //Feedback and Results for the action
    vehicle_auto_control::DynamicLawnmowerFeedback feedback;
    vehicle_auto_control::DynamicLawnmowerResult result;

    //Rate at which to run the control loop
    ros::Rate r(loopHertz);
    bool operating = true;

    
    int currentTrack = goal->currentTrack;
    int currentSection = goal->currentSection;
    
    tf::Vector3 startLocation;
    startLocation.setX(goal->startLocation.x);
    startLocation.setY(goal->startLocation.y);
    startLocation.setZ(goal->startLocation.z);

    int sectionsUnderThreshold = 0;
    std::vector<double> sectionAverages;


    bool trackUnderThreshold = true;
    int sectionsCompletedInTrack = 0;

    int lastTrack = currentTrack - 1;

    tf::Vector3 currentPoint = getPoint(startLocation, 
                                        goal->trackSpacing, 
                                        goal->alongTrackDirection,
                                        goal->acrossTrackDirection,
                                        currentTrack, 
                                        currentSection);

    
    ros::Time lastTime = ros::Time::now();
    while(operating && ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/world", "/" + vehicleName,
                                      ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/world", "/" + vehicleName,  
                                     ros::Time(0), transform);

            if(isAtPoint(transform, currentPoint, false))
            {
                
                if(lastTrack == currentTrack)
                {
                    data_server::GetPlumeData srv;
                    srv.request.name = vehicleName;
                    srv.request.start_time = lastTime;
                    srv.request.end_time = ros::Time::now();
                    plumeClient.call(srv);
                    
                    lastTime = ros::Time::now();

                    //process the plume data
                    bool overThresh = processData(srv.response.plume_val, sectionAverages, goal->continueThreshold);

                    //Track how many sections have been under the threshold and the averages of those sections
                    if(!overThresh)
                    {
                            sectionsUnderThreshold++;
                    }
                    else
                    {
                        trackUnderThreshold = false;
                        sectionsUnderThreshold = 0;
                    }
                    ROS_INFO("SECTIONS UNDER THRESH: %i", sectionsUnderThreshold);
                }
                
                //update last track information
                lastTrack = currentTrack;

                //Make sure a specified number of sections have been completed on this track before going to the next
                //Go to next track or finish the lawnmower if needed, also go to next track if at the edge of the survey area
                bool nextTrack = true;

                if(sectionsCompletedInTrack >= goal->minSectionsPerTrack &&
                   sectionsUnderThreshold >= goal->trackSectionThreshold)
                {
                    //Determines if the average for each section is less than the last.
                    //This prevents the vehicle from turning if heading towards more plume
                    for(unsigned int i = sectionAverages.size() - goal->trackSectionThreshold; i < sectionAverages.size() - 1; i++)
                    {
                        if(sectionAverages[i] < sectionAverages[i + 1])
                        {
                            nextTrack = false;
                            break;
                        }
                    }
                }
                else
                {
                    nextTrack = false;
                }

                //Update current section and current track accordingly
                if(nextTrack || (currentTrack % 2 == 1 && currentSection == 0))
                {
                    currentTrack++;
                    ROS_INFO("CURRENT TRACK: %i", currentTrack);
                    if(trackUnderThreshold)
                    {
                        operating = false;
                    }
                    else
                    {
                        //reset the consecutive sections under the threshold
                        sectionsUnderThreshold = 0;
                        trackUnderThreshold = true;

                        sectionsCompletedInTrack = 0;
                        sectionAverages.clear();
                    }   
                }
                else
                {
                    if(currentTrack % 2 == 0)
                    {
                        currentSection++;
                    }
                    else
                    {
                        currentSection--;
                    }
                    
                    ROS_INFO("CURRENT SECTION: %i", currentSection);

                    sectionsCompletedInTrack++;
                }                  

                currentPoint = getPoint(startLocation, 
                                        goal->trackSpacing, 
                                        goal->alongTrackDirection,
                                        goal->acrossTrackDirection,
                                        currentTrack, 
                                        currentSection);
            }

            //send feedback
            feedback.currentTrack = currentTrack;
            feedback.currentSection = currentSection;
            as->publishFeedback(feedback);
            
            //handle preempt request
            if(as->isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Auto Controller: Dynamic Lawnmower Action Preempted");
                trackUnderThreshold = false; //prevents the actions from declaring success when preempted
                //Stop vehicle
                sendVelocityCommand(0,0,0,0);
                as->setPreempted();
                break;
            }

            goToPoint(transform, currentPoint, goal->targetHeight);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        r.sleep();
    }

    if(trackUnderThreshold)
    {
        ROS_INFO("Auto Controller: Dynamic Lawnmower Action Done, Succeeded");
        
        //Stop vehicle
        sendVelocityCommand(0,0,0,0);

        result.totalTrackLines = currentTrack;
        
        as->setSucceeded(result);
    }

}

bool FourDOFPropulsionController::processData(std::vector<float>& values, std::vector<double>& sectionAverages, double continueThreshold)
{
    bool overThresh = false;
    double averageVal = 0;
    for(unsigned int i = 0; i < values.size(); i++)
    {
        if(values[i] >= continueThreshold)
        {
            overThresh = true;
        }
        averageVal += values[i];
    }
    
    averageVal /= values.size();
    sectionAverages.push_back(averageVal);

    return overThresh;
}

void FourDOFPropulsionController::transformPointToVehicleFrame(geometry_msgs::PointStamped& pointOut, tf::StampedTransform& transform, tf::Vector3& point)
{
    geometry_msgs::PointStamped pointIn;
    
    pointIn.header.stamp = transform.stamp_;
    pointIn.header.frame_id = "/world";
    pointIn.point.x = point.getX();
    pointIn.point.y = point.getY();
    pointIn.point.z = point.getZ();

    listener.transformPoint("/" + vehicleName, pointIn, pointOut);
}

bool FourDOFPropulsionController::isAtPoint(tf::Transform& location, tf::Vector3& point, bool useZ)
{
    double targetVertPosition = std::max(point.getZ(), latestVehicleDepth - latestSonarDepth + minSeafloorDistance);

    double xDifference = fabs(location.getOrigin().getX() - point.getX());
    double yDifference = fabs(location.getOrigin().getY() - point.getY());
    double zDifference = fabs(location.getOrigin().getZ() - targetVertPosition);

    return (!useZ || zDifference <= verticalError) && sqrt(yDifference * yDifference + xDifference * xDifference) <= lateralError;
}

void FourDOFPropulsionController::goToPoint(tf::StampedTransform& location, tf::Vector3& point, double targetHeight)
{
    geometry_msgs::PointStamped pointOut;
    transformPointToVehicleFrame(pointOut, location, point);
    tf::Vector3 vehicleForward(1, 0, 0);
    tf::Vector3 targetPoint(pointOut.point.x, pointOut.point.y, 0);
    tf::Vector3 cross = vehicleForward.cross(targetPoint);

    double angle = vehicleForward.angle(targetPoint);       

    double newVertVel = scaleVerticalVelocity(location, targetHeight);
    double newRotVel = scaleRotationalVelocity(angle, cross.getZ());
    double newForwVel = scaleHorizontalVelocity(location, point);

    sendVelocityCommand(newForwVel, 0, newRotVel, newVertVel);
}

tf::Vector3 FourDOFPropulsionController::getPoint(const tf::Vector3& startLocation, 
                                                  const double sectionSize, 
                                                  const double alongTrackDirection, 
                                                  const double acrossTrackDirection, 
                                                  const int currentTrack, 
                                                  const int currentSection)
{
    tf::Vector3 point;

    //Calculate across track location
    point.setX(startLocation.getX() + cos(acrossTrackDirection) * sectionSize * currentTrack);
    point.setY(startLocation.getY() + sin(acrossTrackDirection) * sectionSize * currentTrack);

    //Add along track location to across track location
    point.setX(point.getX() + cos(alongTrackDirection) * sectionSize * currentSection);
    point.setY(point.getY() + sin(alongTrackDirection) * sectionSize * currentSection);


    point.setZ(startLocation.getZ());

    return point;
}

double FourDOFPropulsionController::scaleHorizontalVelocity(tf::Transform& location, tf::Vector3& point)
{
    double xDifference = fabs(location.getOrigin().getX() - point.getX());
    double yDifference = fabs(location.getOrigin().getY() - point.getY());
    double xyError = sqrt(yDifference * yDifference + xDifference * xDifference);

    double horizontalScaleError = 100;

    if(xyError >= horizontalScaleError)
    {
        return targetHorzVelocity;
    }
    
    return targetHorzVelocity * (xyError / horizontalScaleError);
}

double FourDOFPropulsionController::scaleVerticalVelocity(tf::Transform& location, double targetHeight)
{
    double targetVertPosition = std::max(targetHeight, latestVehicleDepth - latestSonarDepth + minSeafloorDistance);
    double zDifference = fabs(location.getOrigin().getZ() - targetVertPosition);
    double verticalErrorScale = 15;

    int sign = 0;
    if(targetVertPosition >= location.getOrigin().getZ())
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }

    if(zDifference >= verticalErrorScale)
    {
        return targetVertVelocity * sign;
    }
    
    return targetVertVelocity * (zDifference / verticalErrorScale) * sign;
}

double FourDOFPropulsionController::scaleRotationalVelocity(double angleError, double crossZ)
{
    double angleErrorScale = M_PI; //60 degrees 

    if(angleError >= angleErrorScale)
    {
        if(crossZ >= 0)
        {
            return targetRotVelocity;
        }
        else
        {
            return -targetRotVelocity;
        }
    }

    if(crossZ >= 0)
    {
        return targetRotVelocity * (angleError / angleErrorScale);
    }
    
    return -targetRotVelocity * (angleError / angleErrorScale);
}

void FourDOFPropulsionController::sendVelocityCommand(double cmdForwardVelocity, double cmdLateralVelocity, double cmdRotVelocity, double cmdVertVelocity)
{
    geometry_msgs::Twist commandMsg;
    geometry_msgs::Vector3 lin;
    geometry_msgs::Vector3 rot;

    lin.x = cmdForwardVelocity;
    lin.y = cmdLateralVelocity;
    lin.z = cmdVertVelocity;

    rot.x = 0;
    rot.y = 0;
    rot.z = cmdRotVelocity;

    commandMsg.linear = lin;
    commandMsg.angular = rot;
    velocityPub.publish(commandMsg);
}