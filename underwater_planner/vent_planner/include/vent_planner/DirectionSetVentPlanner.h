#ifndef DIRECTION_SET_VENT_PLANNER_H
#define DIRECTION_SET_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"

class DirectionSetVentPlanner : public Planner
{
    
public:
    struct Parameters; //Forward declard Parameters so we can use it in the constructor

    DirectionSetVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters);
    ~DirectionSetVentPlanner() {}

    void receivePlumeData(const PlannerData& data);

    std::shared_ptr<Plan> plan();

private:

    bool endTransect();

public:
    struct Parameters 
    {
        double spiralSpacing;
        double failTime;
        double detectionThreshold;
        double minLegLength;
        double maxLegLength;
        double newMaxThreshold;
        double legSectionLength;
        double numSectionsThreshold;
    };

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
    GoalStatus goalStatus;
    std::unique_ptr<VehicleInterface> vehicleInterface;

    //Vehicle Data
    std::vector<PlannerData> currentData;
    DataNode spiralData;
    bool receivingData;

    //Planner Varaibles
    double plumeHeight;
    VehiclePose currentLineCenter;
    PlannerData currentMax;
    double currentHeading;
    unsigned int numMaxCrossings;

    std::shared_ptr<const Plan> transectPlan;
    SearchPhase currentPlannerStage;

    //Planner Parameters
    Parameters parameters;
};

#endif