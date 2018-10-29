#ifndef DIRECTION_SET_VENT_PLANNER_H
#define DIRECTION_SET_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/controllers/PointPathController.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

class DirectionSetVentPlanner : public Planner
{
public:
    DirectionSetVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, VehicleInfo vehicleInfo);
    ~DirectionSetVentPlanner() {}

    void receivePlumeData(const data_server::PlumeData::ConstPtr& msg);

    std::shared_ptr<Plan> plan();

private:

    void publishGoal();
    void updateGoal();

    bool endTransect();

    /**
    *Sets the parameter returnEntry to the latest data from the vehicle
    *@param returnEntry Output for the latest data
    *@return True if getting the latest data was successful
    **/
    bool getLatestData(DataServerEntry& returnEntry);

private:

    enum SearchPhase { INITIAL_PLAN, 
                       SPIRAL, 
                       EXECUTE_LINE_0, 
                       EXECUTE_LINE_1, 
                       OBSERVE_EXECUTE_LINE_1, 
                       EXECUTE_LINE_2, 
                       EXECUTE_LINE_3, 
                       OBSERVE_EXECUTE_LINE_3};

    std::unique_ptr<VentActionFactory> actionFactory;  
    GoalState goalState;
    VehicleInfo vehicleInfo;
    ros::NodeHandle& nh;

    //Vehicle Data
    ros::ServiceClient latestDataClient;
    ros::Subscriber dataSub;
    std::vector<tf::Vector3> currentData;
    DataNode spiralData;

    //Planner Varaibles
    double plumeHeight;
    tf::Vector3 currentLineCenter;
    tf::Vector3 currentMax;
    double currentHeading;
    unsigned int numMaxCrossings;

    std::shared_ptr<const Plan> transectPlan;
    SearchPhase currentPlannerStage;

    //Planner Parameters
    double spiralSpacing;
    double failTime;
    double detectionThreshold;
    double minLegLength;
    double maxLegLength;
    double newMaxThreshold;
    double legSectionLength;
    double numSectionsThreshold;

    ros::Publisher goalPub;

    PointPathController pointPathController;
};

#endif