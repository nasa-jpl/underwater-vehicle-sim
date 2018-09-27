#ifndef DYNAMIC_LAWNMOWER_CONTROLLER_H
#define DYNAMIC_LAWNMOWER_CONTROLLER_H

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include "vehicle_auto_control/PointPathRosAction.h"
#include "vent_planner/DynamicLawnmowerRosAction.h"

#include "tf/LinearMath/Vector3.h"

class DynamicLawnmowerController
{
public:
    DynamicLawnmowerController(ros::NodeHandle& nh, 
                               std::string vehicleName);
    
    ~DynamicLawnmowerController() {}

private:
    void dynamicLawnmowerUpdate(void);
    void goalCB(void);
    void preemptCB(void);
    
    void pointPathActive(void);
    void pointPathFeedback(const vehicle_auto_control::PointPathRosFeedbackConstPtr& feedback);
    void pointPathDone(const actionlib::SimpleClientGoalState& state,
                       const vehicle_auto_control::PointPathRosResultConstPtr& result);

    void sendPointPathGoal(const std::vector<tf::Vector3>& points);
    void sendPointPathGoal(const tf::Vector3& point);

    /**
    *Processes the data for the dynamic lawnmower action
    */
    bool processData(std::vector<float>& values, double continueThreshold);

    tf::Vector3 getPoint(const tf::Vector3& startLocation,
                     const double sectionSize,
                     const double alongTrackDirection,
                     const double acrossTrackDirection,
                     const int currentTrack,
                     const int currentSection);

private:
    actionlib::SimpleActionServer<vent_planner::DynamicLawnmowerRosAction> dynamicLawnmowerServer;
    actionlib::SimpleActionClient<vehicle_auto_control::PointPathRosAction> pointPathClient;
    ros::ServiceClient plumeClient;

    bool replanNextUpdate;

    tf::Vector3 startLocation;
    double alongTrackDirection;
    double acrossTrackDirection;
    double trackSpacing;
    double targetHeight;
    int minSectionsPerTrack;
    double continueThreshold;
    int trackSectionThreshold;

    int currentTrack;
    int currentSection; 
    int lastTrack;

    ros::Time lastTime;

    int sectionsUnderThreshold;

    bool trackUnderThreshold;
    int sectionsCompletedInTrack;

    std::vector<double> sectionAverages;

    std::string vehicleName;
};

#endif
