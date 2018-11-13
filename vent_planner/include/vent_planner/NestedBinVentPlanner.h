#ifndef NESTED_BIN_VENT_PLANNER_H
#define NESTED_BIN_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"

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

    enum SearchPhase {none, spiral, dynamic, nested };

    void receivePlumeData(const PlannerData& data);

    void publishLog(std::string log);

    DataNode getLatestSpiralData();
    bool newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold);

    void initalizeDataTree(VehiclePose centerLocation);

    std::set<DataNode*, DataNode::PointerCompare> getUnexploredMaxima();
    void addInitalLawnmowers(std::shared_ptr<Plan> plan, const VehiclePose& centerLocation, double plumeHeight);


    bool isGoalSurvey(double nestedBinSize, DataNode* maximum, std::vector<DataNode*>& neighbors);

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

    std::unique_ptr<DataTree> dataTree;
    DataNode spiralData;

    std::map<std::shared_ptr<Plan>, DataNode*> plannedMaxima;

    bool receivingData;

    std::shared_ptr<Plan> finalSurvey;

    //Planner Parameters
    Parameters parameters;
};

#endif