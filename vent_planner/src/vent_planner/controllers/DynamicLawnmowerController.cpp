#include "vent_planner/controllers/DynamicLawnmowerController.h"

#include "data_server/GetPlumeData.h"

DynamicLawnmowerController::DynamicLawnmowerController(ros::NodeHandle& nh, 
                                                       std::string vehicleName) :
    pointPathClient(nh, "vehicle_controller/" + vehicleName + "/point_path", false),
    dynamicLawnmowerServer(nh, "planner/" + vehicleName + "/dynamic_lawnmower", false),
    plumeClient(nh.serviceClient<data_server::GetPlumeData>("data_server/get_plume")),
    vehicleName(vehicleName)
{

    //Init action server
    dynamicLawnmowerServer.registerGoalCallback(boost::bind(&DynamicLawnmowerController::goalCB, this));
    dynamicLawnmowerServer.registerPreemptCallback(boost::bind(&DynamicLawnmowerController::preemptCB, this));
    dynamicLawnmowerServer.start();
}
    
void DynamicLawnmowerController::dynamicLawnmowerUpdate(void)
{
    tf::Vector3 currentPoint = getPoint(startLocation,
                                        trackSpacing,
                                        alongTrackDirection,
                                        acrossTrackDirection,
                                        currentTrack,
                                        currentSection);
    sendPointPathGoal(currentPoint);
}

void DynamicLawnmowerController::goalCB(void)
{
    pointPathClient.cancelAllGoals();
    vent_planner::DynamicLawnmowerRosGoalConstPtr dynamicLawnmowerGoal = 
        dynamicLawnmowerServer.acceptNewGoal();

    startLocation.setX(dynamicLawnmowerGoal->startLocation.x);
    startLocation.setY(dynamicLawnmowerGoal->startLocation.y);
    startLocation.setZ(dynamicLawnmowerGoal->startLocation.z);

    currentTrack = dynamicLawnmowerGoal->currentTrack;
    currentSection = dynamicLawnmowerGoal->currentSection;

    lastTrack = currentTrack - 1;

    alongTrackDirection = dynamicLawnmowerGoal->alongTrackDirection;
    acrossTrackDirection = dynamicLawnmowerGoal->acrossTrackDirection;
    trackSpacing = dynamicLawnmowerGoal->trackSpacing;
    targetHeight = dynamicLawnmowerGoal->targetHeight;
    minSectionsPerTrack = dynamicLawnmowerGoal->minSectionsPerTrack;
    continueThreshold = dynamicLawnmowerGoal->continueThreshold;
    trackSectionThreshold = dynamicLawnmowerGoal->trackSectionThreshold;

    sectionsUnderThreshold = 0;
    sectionAverages.clear();

    trackUnderThreshold = true;
    sectionsCompletedInTrack = 0;

    lastTime = ros::Time::now();  
    dynamicLawnmowerUpdate();   
}

void DynamicLawnmowerController::preemptCB(void)
{
    pointPathClient.cancelAllGoals();
    dynamicLawnmowerServer.setPreempted();
}
    
void DynamicLawnmowerController::pointPathActive(void)
{
}

void DynamicLawnmowerController::pointPathFeedback(const vehicle_auto_control::PointPathRosFeedbackConstPtr& feedback)
{
}

void DynamicLawnmowerController::pointPathDone(const actionlib::SimpleClientGoalState& state,
                       const vehicle_auto_control::PointPathRosResultConstPtr& result)
{
    bool dynamicLawnmowerComplete = false;

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Dynamic lawnmower point path succeeded");

        //Update section over theshold information
        if(lastTrack == currentTrack)
        {
            data_server::GetPlumeData srv;
            srv.request.name = vehicleName;
            srv.request.start_time = lastTime;
            srv.request.end_time = ros::Time::now();
            plumeClient.call(srv); //TODO: Init this

            lastTime = ros::Time::now();

            //process the plume data
            bool overThresh = processData(srv.response.plume_val, continueThreshold);

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
            ROS_DEBUG("Dynamic lawnmower sections under threshold: %i", sectionsUnderThreshold);
        }

        //update last track information
        lastTrack = currentTrack;

        //Make sure a specified number of sections have been completed on this track before going to the next
        //Go to next track or finish the lawnmower if needed, also go to next track if at the edge of the survey area
        bool nextTrack = true;

        if(sectionsCompletedInTrack >= minSectionsPerTrack &&
           sectionsUnderThreshold >= trackSectionThreshold)
        {
            //Determines if the average for each section is less than the last.
            //This prevents the vehicle from turning if heading towards more plume
            for(unsigned int i = sectionAverages.size() - trackSectionThreshold; i < sectionAverages.size() - 1; i++)
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
            ROS_DEBUG("Dynamic lawnmower current track: %i", currentTrack);
            if(!trackUnderThreshold)
            {
                //reset the consecutive sections under the threshold
                sectionsUnderThreshold = 0;
                trackUnderThreshold = true;

                sectionsCompletedInTrack = 0;
                sectionAverages.clear();
            }
            else
            {
                dynamicLawnmowerComplete = true;
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

            ROS_INFO("Dynamic lawnmower current section: %i", currentSection);

            sectionsCompletedInTrack++;
        }
    }
    else
    {
        ROS_INFO("Dynamic lawnmower point path failed");
    }

    if(dynamicLawnmowerComplete)
    {
        ROS_INFO("Dynamic Lawnmower action set succeeded");
        vent_planner::DynamicLawnmowerRosResult result;
        result.totalTrackLines = currentTrack;
        dynamicLawnmowerServer.setSucceeded(result);
    }
    else
    {
       //update info based on what point path returns
        dynamicLawnmowerUpdate(); 
    }
}

void DynamicLawnmowerController::sendPointPathGoal(const tf::Vector3& point)
{
    std::vector<tf::Vector3> points;
    points.push_back(point);
    sendPointPathGoal(points);
}

void DynamicLawnmowerController::sendPointPathGoal(const std::vector<tf::Vector3>& points)
{
    //Creates an action goal and sends it to the action server for point path movement
    vehicle_auto_control::PointPathRosGoal pointPathGoal = vehicle_auto_control::PointPathRosGoal();

    for(tf::Vector3 point : points)
    {
        geometry_msgs::Point p;
        p.x = point.getX();
        p.y = point.getY();
        p.z = point.getZ();
        pointPathGoal.points.push_back(p);
    }

    pointPathGoal.upperDepth = 0;
    pointPathGoal.lowerDepth = 0;
    pointPathGoal.yoyo = false;

    ROS_INFO("Dynamic lawnmower send goal to point path server");
    pointPathClient.waitForServer();
    pointPathClient.sendGoal(pointPathGoal,
        boost::bind(&DynamicLawnmowerController::pointPathDone, this, _1, _2),
        boost::bind(&DynamicLawnmowerController::pointPathActive, this),
        boost::bind(&DynamicLawnmowerController::pointPathFeedback, this, _1));
}

tf::Vector3 DynamicLawnmowerController::getPoint(const tf::Vector3& startLocation,
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

bool DynamicLawnmowerController::processData(std::vector<float>& values, double continueThreshold)
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
