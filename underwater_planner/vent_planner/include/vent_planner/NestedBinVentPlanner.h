#ifndef NESTED_BIN_VENT_PLANNER_H
#define NESTED_BIN_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/DynamicLawnmower.h"
#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"

class NestedBinVentPlanner : public Planner
{
public:
    struct Parameters; //Forward declard Parameters so we can use it in the constructor

    NestedBinVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters);
    ~NestedBinVentPlanner() {}

    std::shared_ptr<Plan> plan();

private:

    void receivePlumeData(const PlannerData& data);

    void publishLog(std::string log);

    DataNode getLatestSpiralData();
    bool newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold);

    void initalizeDataTree(VehiclePose centerLocation);

    std::set<DataNode*, DataNode::PointerCompare> getUnexploredMaxima();
    void initializeDynamicLawnmower();


    bool isGoalSurvey(double nestedBinSize, DataNode* maximum, std::vector<DataNode*>& neighbors);

    enum SearchPhase { SPIRAL,
                       OBSERVE_SPIRAL,
                       START_NEXT_LAWNMOWER,
                       START_DYNAMIC_LAWNMOWER,
                       RUN_DYNAMIC_LAWNMOWER};

public:
    struct Parameters 
    {
        double spiralSpacing;
        double initalSpacing;
        double finalSpacing;
        double failTime;
    };

private:
    std::unique_ptr<VentActionFactory> actionFactory;
    std::unique_ptr<VehicleInterface> vehicleInterface;

    SearchPhase phase;

    std::shared_ptr<Plan> spiralPlan;
    std::shared_ptr<Plan> dynamicPlan;

    std::unique_ptr<DynamicLawnmower> dynamicLawnmower;
    VehiclePose dynamicLawnmowerCenter;
    unsigned int currentDynamicLawnmower;
    double plumeHeight;
    std::vector<PlannerData> dynamicLawnmowerSectionData;

    std::unique_ptr<DataTree> dataTree;
    DataNode spiralData;

    std::map<std::shared_ptr<Plan>, DataNode*> plannedMaxima;

    bool receivingData;

    std::shared_ptr<Plan> finalSurvey;

    //Planner Parameters
    Parameters parameters;
};

#endif