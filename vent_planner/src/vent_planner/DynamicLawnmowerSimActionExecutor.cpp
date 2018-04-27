#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Point.h"

#include "planner_framework/Action.h"

#include "vent_planner/DynamicLawnmowerSimActionExecutor.h"
#include "vent_planner/actions/DynamicLawnmowerAction.h"
#include "vehicle_auto_control/Velocity.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"

#include "vent_planner/ExecuteDynamicLawnmowerAction.h"
#include "vehicle_auto_control/DynamicLawnmowerAction.h"

#include "data_server/GetPlumeData.h"
#include "vehicle_auto_control/PointPathAction.h"

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(ros::NodeHandle& nh, std::string vehicleName, double loopHertz) :
	vehicleName(vehicleName),
	nh(nh),
	dynamicLawnmowerClient("planner/"  + vehicleName + "/dynamic_lawnmower", false),
    pointPathClient("vehicle_controller/"  + vehicleName + "/point_path", false),
	velPublisher(nh.advertise<vehicle_auto_control::Velocity>("/vehicle_controller/" + vehicleName + "/command_target_velocity", 1000, true)),
    actionServer(nh, "planner/"  + vehicleName + "/dynamic_lawnmower", boost::bind(&DynamicLawnmowerSimActionExecutor::executeAction, this, _1, &actionServer), false),
    loopHertz(loopHertz),
    plumeClient(nh.serviceClient<data_server::GetPlumeData>("data_server/get_plume"))
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("/vehicles/get_info");
	infoClient.waitForExistence();

    actionServer.start();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

DynamicLawnmowerSimActionExecutor::DynamicLawnmowerSimActionExecutor(const DynamicLawnmowerSimActionExecutor& other) :
	vehicleName(other.vehicleName),
	nh(other.nh),
	dynamicLawnmowerClient("planner/"  + vehicleName + "/dynamic_lawnmower", false),
    pointPathClient("vehicle_controller/"  + vehicleName + "/point_path", false),
    actionServer(nh, "planner/"  + vehicleName + "/dynamic_lawnmower", boost::bind(&DynamicLawnmowerSimActionExecutor::executeAction, this, _1, &actionServer), false),
    loopHertz(other.loopHertz),
    plumeClient(nh.serviceClient<data_server::GetPlumeData>("data_server/get_plume"))
{
	infoClient = nh.serviceClient<underwater_vehicle_sim::GetVehicleInfo>("/vehicles/get_info");
	infoClient.waitForExistence();

    actionServer.start();

	underwater_vehicle_sim::GetVehicleInfo info;
	info.request.name = vehicleName;
	infoClient.call(info);
	vehicleInfo = info.response;
}

std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> DynamicLawnmowerSimActionExecutor::clone()
{
	std::unique_ptr<ActionExecutor<DynamicLawnmowerAction>> a(new DynamicLawnmowerSimActionExecutor(*this));
    return a;
}


bool DynamicLawnmowerSimActionExecutor::execute(std::shared_ptr<DynamicLawnmowerAction> action)
{
	//targetSlope can only be on the interval (0, 90) degrees
	if(action->targetSlope >= M_PI / 2 || action->targetSlope <= 0)
	{
		return false;
	}

	if(vehicleInfo.propModuleType == "FourDOFPropulsion")
	{
		//Send target velocities command
		vehicle_auto_control::Velocity velMsg;
		velMsg.horizontalVelocity = action->targetHorizontalVelocity;

		//Calculate the target vertical velocity based on target horizontal velocity and target slope
		velMsg.verticalVelocity = action->targetHorizontalVelocity * (sin(action->targetSlope) / cos(action->targetSlope));

		velMsg.rotationalVelocity = action->targetRotationalVelocity;
	
		velPublisher.publish(velMsg);

	}
	else //If the prop module is not known then this cannot be completed
	{
		return false;
	}
	//Creates an action goal and sends it to the action server for point path movement
	dynamicLawnmowerGoal = vent_planner::ExecuteDynamicLawnmowerGoal();

	dynamicLawnmowerGoal.startLocation.x = action->startLocation.getX();
	dynamicLawnmowerGoal.startLocation.y = action->startLocation.getY();
	dynamicLawnmowerGoal.startLocation.z = action->startLocation.getZ();

	dynamicLawnmowerGoal.currentTrack = action->getCurrentTrack();
	dynamicLawnmowerGoal.currentSection = action->getCurrentSection();
	dynamicLawnmowerGoal.alongTrackDirection = action->alongTrackDirection;
	dynamicLawnmowerGoal.acrossTrackDirection = action->acrossTrackDirection;
	dynamicLawnmowerGoal.trackSpacing = action->trackSpacing;
	dynamicLawnmowerGoal.targetHeight = action->targetHeight;

	dynamicLawnmowerGoal.minSectionsPerTrack = action->minSectionsPerTrack;
	dynamicLawnmowerGoal.continueThreshold = action->continueThreshold;
	dynamicLawnmowerGoal.trackSectionThreshold = action->trackSectionThreshold;

	dynamicLawnmowerClient.waitForServer();
	dynamicLawnmowerClient.sendGoal(dynamicLawnmowerGoal,
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionDone, this, action, _1, _2),
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionActive, this, action),
							 boost::bind(&DynamicLawnmowerSimActionExecutor::actionFeedback, this, action, _1));

	//replan at the start of each action, i.e. the end of the previous action
	replanNextUpdate = true;

	return true;
}

void DynamicLawnmowerSimActionExecutor::executeAction(const vent_planner::ExecuteDynamicLawnmowerGoalConstPtr& goal,
													  actionlib::SimpleActionServer<vent_planner::ExecuteDynamicLawnmowerAction>* as)
{
    //Feedback and Results for the action
    vent_planner::ExecuteDynamicLawnmowerFeedback feedback;
    vent_planner::ExecuteDynamicLawnmowerResult result;

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

    ros::Time lastTime = ros::Time::now();

    bool waitingForCommand = true;

    while(operating && ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/world", "/" + vehicleName,
                                      ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/world", "/" + vehicleName,
                                     ros::Time(0), transform);


            if(waitingForCommand)
            {
                tf::Vector3 currentPoint = getPoint(startLocation,
                                        goal->trackSpacing,
                                        goal->alongTrackDirection,
                                        goal->acrossTrackDirection,
                                        currentTrack,
                                        currentSection);

                if(operating)
                {
                    sendPointPathGoal(currentPoint);
                    ROS_INFO("Planner: Sent Dynamic Lawnmower Point Path Goal");
                    waitingForCommand = false;
                }


            }
            else
            {
                actionlib::SimpleClientGoalState state = pointPathClient.getState();

                if(as->isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("Planner: Dynamic Lawnmower Action Preempted");
                    trackUnderThreshold = false; //prevents the actions from declaring success when preempted

                    //Stop vehicle
                    pointPathClient.cancelAllGoals();
                    as->setPreempted();
                    break;
                }
                else if(state == actionlib::SimpleClientGoalState::ABORTED ||
                        state == actionlib::SimpleClientGoalState::REJECTED ||
                        state == actionlib::SimpleClientGoalState::RECALLED)
                {

                    waitingForCommand = true;
                }
                else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Planner: Point Path Succeeded");
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

                    if(operating)
                    {
                        waitingForCommand = true;
                    }


                }

                //send feedback
                feedback.currentTrack = currentTrack;
                feedback.currentSection = currentSection;
                as->publishFeedback(feedback);
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        r.sleep();
    }

    if(trackUnderThreshold)
    {
        ROS_INFO("Auto Controller: Dynamic Lawnmower Action Done, Succeeded");

        result.totalTrackLines = currentTrack;

        as->setSucceeded(result);
    }
}

void DynamicLawnmowerSimActionExecutor::sendPointPathGoal(const tf::Vector3& point)
{
    std::vector<tf::Vector3> points;
    points.push_back(point);
    sendPointPathGoal(points);
}

void DynamicLawnmowerSimActionExecutor::sendPointPathGoal(const std::vector<tf::Vector3>& points)
{
    //Creates an action goal and sends it to the action server for point path movement
    vehicle_auto_control::PointPathGoal pointPathGoal = vehicle_auto_control::PointPathGoal();

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

    pointPathClient.waitForServer();
    pointPathClient.sendGoal(pointPathGoal);
}

tf::Vector3 DynamicLawnmowerSimActionExecutor::getPoint(const tf::Vector3& startLocation,
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

bool DynamicLawnmowerSimActionExecutor::processData(std::vector<float>& values, std::vector<double>& sectionAverages, double continueThreshold)
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

void DynamicLawnmowerSimActionExecutor::cancel(std::shared_ptr<DynamicLawnmowerAction> action)
{
	dynamicLawnmowerClient.cancelAllGoals();
}

bool DynamicLawnmowerSimActionExecutor::triggerReplan(std::shared_ptr<DynamicLawnmowerAction> action)
{
	bool replanReturn = replanNextUpdate;
	replanNextUpdate = false;
	return replanReturn;
}

void DynamicLawnmowerSimActionExecutor::actionDone(std::shared_ptr<DynamicLawnmowerAction> action,
					const actionlib::SimpleClientGoalState& state,
                	const vent_planner::ExecuteDynamicLawnmowerResultConstPtr& result)
{
	ROS_INFO("Planner: DynamicLawnmower ActionDone Start");
	if(state == actionlib::SimpleClientGoalState::RECALLED ||
	   state == actionlib::SimpleClientGoalState::PREEMPTED)
	{
		action->setState(Action::State::INTERRUPTED);
	}
	else if(state == actionlib::SimpleClientGoalState::REJECTED ||
			state == actionlib::SimpleClientGoalState::ABORTED)
	{
		action->setState(Action::State::FAILED);
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		action->setState(Action::State::COMPLETED);
	}

	ROS_INFO("Planner: DynamicLawnmower ActionDone End");
}

void DynamicLawnmowerSimActionExecutor::actionActive(std::shared_ptr<DynamicLawnmowerAction> action)
{
	action->setState(Action::State::EXECUTING);
}

void DynamicLawnmowerSimActionExecutor::actionFeedback(std::shared_ptr<DynamicLawnmowerAction> action,
					const vent_planner::ExecuteDynamicLawnmowerFeedbackConstPtr& feedback)
{
	action->setCurrentTrack(feedback->currentTrack);
	action->setCurrentSection(feedback->currentSection);
}