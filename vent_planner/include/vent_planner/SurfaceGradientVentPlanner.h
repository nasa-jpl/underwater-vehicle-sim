#ifndef SURFACE_GRADIENT_VENT_PLANNER_H
#define SURFACE_GRADIENT_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "planner_framework/VehicleInterface.h"
#include "planner_framework/Planner.h"
#include "planner_framework/VehiclePose.h"
#include "planner_framework/GoalStatus.h"

#include "vent_planner/DataNode.h"
#include "vent_planner/actions/VentActionFactory.h"

class SurfaceGradientVentPlanner : public Planner
{
public:
    struct Parameters; //Forward declard Parameters so we can use it in the constructor

    SurfaceGradientVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters);
    ~SurfaceGradientVentPlanner() override = default;

    void receivePlumeData(const PlannerData& data);

    std::shared_ptr<Plan> plan() override;

private:
    bool endGradientFollow();

public:
    struct Parameters
    {
        double spiralSpacing;
        double failTime;
        double detectionThreshold;
        double gradientCalcRadius;
        double gradientMinFollowDistance;
        double gradientMaxFollowDistance;
        double gradientThreshold;
        double gradientWindow;
    };

private:

    enum SearchPhase { INITIAL_PLAN, SPIRAL, PLAN_GRADIENT, CALC_GRADIENT, FOLLOW_GRADIENT, OBSERVE_FOLLOW_GRADIENT};

    std::unique_ptr<VentActionFactory> actionFactory; 
    std::unique_ptr<VehicleInterface> vehicleInterface; 

    std::vector<PlannerData> currentData;
    DataNode spiralData;

    double plumeHeight;
    double gradientDirection;

    bool receivingData;
    SearchPhase currentPlannerStage;

    std::shared_ptr<const Plan> gradientCirclePlan;
    std::shared_ptr<const Plan> gradientFollowPlan;
    std::shared_ptr<const PointPathAction> gradientFollowAction;

    VehiclePose gradientLocation;

    Parameters parameters;
};

#endif