#ifndef SURFACE_GRADIENT_VENT_PLANNER_H
#define SURFACE_GRADIENT_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"

class SurfaceGradientVentPlanner : public Planner
{
public:
    SurfaceGradientVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName);
    ~SurfaceGradientVentPlanner() {}

    void receivePlumeData(const data_server::PlumeData::ConstPtr& msg);

    std::shared_ptr<Plan> plan();

private:

    void publishGoal();
    void updateGoal();

    /**
    *Sets the parameter returnEntry to the latest data from the vehicle
    *@param returnEntry Output for the latest data
    *@return True if getting the latest data was successful
    **/
    bool getLatestData(DataServerEntry& returnEntry);

private:

    enum SearchPhase { INITIAL_PLAN, SPIRAL, PLAN_GRADIENT, CALC_GRADIENT, FOLLOW_GRADIENT };

    std::unique_ptr<VentActionFactory> actionFactory;  
    std::string goalState;
    std::string vehicleName;
    ros::NodeHandle& nh;

    ros::ServiceClient latestDataClient;
    ros::Subscriber dataSub;
    std::vector<tf::Vector3> currentData;
    DataNode spiralData;

    double plumeHeight;
    double gradientDirection;

    SearchPhase currentPlannerStage;

    std::shared_ptr<Plan> spiralPlan;
    std::shared_ptr<Plan> gradientPlan;

    tf::Vector3 gradientLocation;

    double spiralSpacing;
    double failTime;
    ros::Publisher goalPub;

    double detectionThreshold;
    double gradientCalcRadius;
    double gradientFollowDistance;
};

#endif