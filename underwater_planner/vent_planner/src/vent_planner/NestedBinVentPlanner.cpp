#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "planner_framework/VehicleInterface.h"
#include "planner_framework/PlannerData.h"
#include "planner_framework/VehiclePose.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/NestedBinVentPlanner.h"

#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"

#include "vent_planner/util/CreatePathUtil.h"

NestedBinVentPlanner::NestedBinVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters) :
    actionFactory(std::move(actionFactory)),
    vehicleInterface(std::move(vehicleInterface)),
    parameters(std::move(parameters)),
    finalSurvey(nullptr),
    phase(SearchPhase::SPIRAL),
    spiralData(nullptr, 0, VehiclePose(0,0,0), 300000, 0),
    receivingData(false),
    currentDynamicLawnmower(0),
    dynamicLawnmower(nullptr),
    plumeHeight(0),
    dynamicLawnmowerCenter(VehiclePose(0,0,0))
{
    this->vehicleInterface->registerDataCallback(std::bind(&NestedBinVentPlanner::receivePlumeData, this, std::placeholders::_1));
}

void NestedBinVentPlanner::receivePlumeData(const PlannerData& data)
{
    if((phase == SearchPhase::START_NEXT_LAWNMOWER) &&
       dataTree)
    {
        dataTree->addData(data);
    }
    else if((phase == SearchPhase::START_DYNAMIC_LAWNMOWER || phase == SearchPhase::RUN_DYNAMIC_LAWNMOWER) &&
             dataTree)
    {
        dataTree->addData(data);
        dynamicLawnmowerSectionData.push_back(data);
    }
    else if(phase == SearchPhase::OBSERVE_SPIRAL)
    {
        spiralData.addData(data);
    }
    receivingData = true;
}

std::shared_ptr<Plan> NestedBinVentPlanner::plan()
{
    vehicleInterface->log(LogLevel::INFO, "Plan");
    //Amount to reduce the bin size each nested pattern
    double nestedSizeFactor = 3;
    double detectionThreshold = 0.5;

    std::shared_ptr<Plan> returnPlan;

    if(phase == SearchPhase::SPIRAL)
    {
        vehicleInterface->log(LogLevel::INFO, "SPIRAL Phase");
        if(receivingData)
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());
            spiralPlan = returnPlan;
            VehiclePose pose = vehicleInterface->getPosition();
            VehiclePose vehicleLocation(pose.getX(), pose.getY(), pose.getZ());
            std::vector<VehiclePose> spiralPoints = create_path_util::makeSpiral(vehicleLocation, 0, parameters.spiralSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     -100,
                                                                                     -2000,
                                                                                     spiralPoints,
                                                                                     PointPathAction::ReplanType::ON_YOYO_TURN,
                                                                                     0);
            returnPlan->addAction(newAction);
            vehicleInterface->log(LogLevel::INFO, "Spiral");

            phase = SearchPhase::OBSERVE_SPIRAL;
        }
    }
    else if(phase == SearchPhase::OBSERVE_SPIRAL)
    {
        vehicleInterface->log(LogLevel::INFO, "OBSERVE_SPIRAL Phase");
        bool valid = newSpiralPlumeIntersect(spiralData, detectionThreshold);

        if(valid)
        {
            PlannerData maxVal = spiralData.getMaxVal();
            plumeHeight = spiralData.getHeightOfPlume();

            initalizeDataTree(maxVal.getPose());
            dynamicLawnmowerCenter = dataTree->getClosestNodeOrigin(maxVal.getPose(), 1);
            returnPlan = std::shared_ptr<Plan>(new Plan());
            phase = SearchPhase::START_DYNAMIC_LAWNMOWER; 
        }
        else
        {
            returnPlan = spiralPlan;
            returnPlan->resetInterrupted();
            phase = SearchPhase::OBSERVE_SPIRAL;
        }

        spiralData.clear();
    }
    else if(phase == START_NEXT_LAWNMOWER)
    {
        vehicleInterface->log(LogLevel::INFO, "START_NEXT_LAWNMOWER Phase");
        std::set<DataNode*, DataNode::PointerCompare> queuedMaxima = getUnexploredMaxima();

        if(queuedMaxima.size() > 0)
        {
            auto lastElement = queuedMaxima.end();
            --lastElement;
            DataNode* maximum = *lastElement;

            double nestedBinSize = maximum->getSize() / nestedSizeFactor;

            std::vector<DataNode*> neighbors = maximum->getInitalizedNeighbors();

            //Check for goal completion.
            bool isFinalSurvey = isGoalSurvey(nestedBinSize, maximum, neighbors);

            if(!maximum->isPartitioned())
            {
                maximum->partition(nestedSizeFactor);
            }

            for(auto neighbor : neighbors)
            {
                if(!neighbor->isPartitioned())
                {
                    neighbor->partition(nestedSizeFactor);
                }
            }

            VehiclePose startLocation(maximum->getCenterLocation().getX() - (maximum->getSize() * 1.5) + (nestedBinSize / 2),
                                      maximum->getCenterLocation().getY() - (maximum->getSize() * 1.5) + (nestedBinSize / 2),
                                      maximum->getHeightOfPlume());

            std::vector<VehiclePose> nestedPattern = create_path_util::makeLawnmower(startLocation,
                                                                   0,
                                                                   M_PI / 2,
                                                                   (nestedSizeFactor * 3 - 1) * nestedBinSize,
                                                                   (nestedSizeFactor * 3 - 1) * nestedBinSize,
                                                                   nestedBinSize);

            std::shared_ptr<PointPathAction> lawnmowerAction = actionFactory->createPointPathAction(1.0,
                                                                                                    0.349066,
                                                                                                    0.523599, //30 deg
                                                                                                    nestedPattern,
                                                                                                    PointPathAction::ReplanType::NONE,
                                                                                                    0);

            returnPlan = std::shared_ptr<Plan>(new Plan());
            returnPlan->addAction(lawnmowerAction);
            plannedMaxima.insert(std::make_pair(returnPlan, maximum));
            phase = SearchPhase::START_NEXT_LAWNMOWER;

            if(isFinalSurvey)
            {
                finalSurvey = returnPlan;
            }

            std::stringstream ss;
            ss.precision(5);
            ss << std::fixed << "NestedLawnmower," << startLocation.getZ() << "," << startLocation.getX() << "," << startLocation.getY() << "," << nestedBinSize;
            vehicleInterface->log(LogLevel::INFO, ss.str());
        }
        else
        {
            if(currentDynamicLawnmower < 4)
            {
                returnPlan = std::shared_ptr<Plan>(new Plan());
                phase = SearchPhase::START_DYNAMIC_LAWNMOWER;
            }
            else
            {
                returnPlan = spiralPlan;
                phase = SearchPhase::OBSERVE_SPIRAL;
            }
        }
    }
    else if(phase == START_DYNAMIC_LAWNMOWER)
    {
        vehicleInterface->log(LogLevel::INFO, "START_DYNAMIC_LAWNMOWER Phase");
        initializeDynamicLawnmower();

        //Create first command for dynamic lawnmower
        returnPlan = std::shared_ptr<Plan>(new Plan());
        VehiclePose nextPoint = dynamicLawnmower->getNextPoint();
        std::vector<VehiclePose> points;
        points.push_back(nextPoint);
        std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                           0.349066,
                                                                           0.523599, //30 deg
                                                                           points,
                                                                           PointPathAction::ReplanType::NONE,
                                                                           0);
        returnPlan->addAction(newAction);

        phase = SearchPhase::RUN_DYNAMIC_LAWNMOWER;

        //Log data to file
        std::stringstream ss;
        ss.precision(5);
        ss << std::fixed << "DynamicLawnmower," << plumeHeight << "," << dynamicLawnmowerCenter.getX() << "," << dynamicLawnmowerCenter.getY() << "," << parameters.initalSpacing;
        vehicleInterface->log(LogLevel::INFO, ss.str());
    }
    else if(phase == RUN_DYNAMIC_LAWNMOWER)
    {
        vehicleInterface->log(LogLevel::INFO, "RUN_DYNAMIC_LAWNMOWER Phase");
        returnPlan = std::shared_ptr<Plan>(new Plan());

        dynamicLawnmower->completeSection(dynamicLawnmowerSectionData);
        dynamicLawnmowerSectionData.clear();

        if(dynamicLawnmower->isDone())
        {
            phase = SearchPhase::START_NEXT_LAWNMOWER;
            currentDynamicLawnmower++;
        }
        else
        {
            VehiclePose nextPoint = dynamicLawnmower->getNextPoint();
            std::vector<VehiclePose> points;
            points.push_back(nextPoint);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                               0.349066,
                                                                               0.523599, //30 deg
                                                                               points,
                                                                               PointPathAction::ReplanType::NONE,
                                                                               0);
            returnPlan->addAction(newAction);
        }
    }

    if(finalSurvey && finalSurvey->isCompleted())
    {
        vehicleInterface->sendGoalStatus(GoalStatus::SUCCESS);
    }

    return returnPlan;
}

bool NestedBinVentPlanner::newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold)
{
    PlannerData maxVal = spiralData.getMaxVal();
    bool intersect = false;
    if(maxVal.getData()["plume"] >= detectionThreshold)
    {
        if(!dataTree)
        {
            intersect = true;
        }
        else
        {
            //Set valid to false if this area has already been investigated
            DataNode& smallestAtMaxVal = dataTree->getSmallestNode(maxVal.getPose());
            if(smallestAtMaxVal.getNodeLevel() < 1)
            {
                intersect = true;
            }
        }
    }

    return intersect;
}

void NestedBinVentPlanner::initalizeDataTree(VehiclePose centerLocation)
{
    if(!dataTree)
    {
        float targetSize = 300000;
        int numPartitions = ceil(targetSize / parameters.initalSpacing);

        dataTree = std::unique_ptr<DataTree>(new DataTree(centerLocation,
                                                          numPartitions * parameters.initalSpacing));
        dataTree->getRoot().partition(numPartitions);
    }
}

std::set<DataNode*, DataNode::PointerCompare> NestedBinVentPlanner::getUnexploredMaxima()
{
    std::set<DataNode*, DataNode::PointerCompare> queuedMaxima;
    const std::vector<DataNode*> binMaxima = dataTree->getMaxima();
    for(unsigned int i = 0; i < binMaxima.size(); i++)
    {
        bool found = false;
        for (auto it = plannedMaxima.begin(); it != plannedMaxima.end(); ++it)
        {
            if(*(it->second) == *binMaxima[i])
            {
                found = true;
                break;
            }
        }

        if(!found && binMaxima[i]->getSize() > parameters.finalSpacing)
        {
            unsigned long queueSize = queuedMaxima.size();
            queuedMaxima.insert(binMaxima[i]);
        }
    }

    return queuedMaxima;
}

void NestedBinVentPlanner::initializeDynamicLawnmower()
{
    if(currentDynamicLawnmower == 0)
    {
        VehiclePose lawnmower0Start(dynamicLawnmowerCenter.getX() + parameters.initalSpacing / 2.0, dynamicLawnmowerCenter.getY() + parameters.initalSpacing / 2.0, plumeHeight);
        dynamicLawnmower.reset(new DynamicLawnmower(lawnmower0Start,
                                                    0,
                                                    M_PI / 2,
                                                    parameters.initalSpacing,
                                                    plumeHeight,
                                                    4,
                                                    0.5,
                                                    2));
    }
    else if(currentDynamicLawnmower == 1)
    {
        VehiclePose lawnmower1Start(dynamicLawnmowerCenter.getX() - parameters.initalSpacing / 2.0, dynamicLawnmowerCenter.getY() + parameters.initalSpacing / 2.0, plumeHeight);
        dynamicLawnmower.reset(new DynamicLawnmower(lawnmower1Start,
                                                    M_PI,
                                                    M_PI / 2,
                                                    parameters.initalSpacing,
                                                    plumeHeight,
                                                    4,
                                                    0.5,
                                                    2));
    }
    else if(currentDynamicLawnmower == 2)
    {
        VehiclePose lawnmower2Start(dynamicLawnmowerCenter.getX() - parameters.initalSpacing / 2.0, dynamicLawnmowerCenter.getY() - parameters.initalSpacing / 2.0, plumeHeight);
        dynamicLawnmower.reset(new DynamicLawnmower(lawnmower2Start,
                                                    M_PI,
                                                    M_PI * 3 / 2,
                                                    parameters.initalSpacing,
                                                    plumeHeight,
                                                    4,
                                                    0.5,
                                                    2));
    }
    else if(currentDynamicLawnmower == 3)
    {
        VehiclePose lawnmower3Start(dynamicLawnmowerCenter.getX() + parameters.initalSpacing / 2.0, dynamicLawnmowerCenter.getY() - parameters.initalSpacing / 2.0, plumeHeight);
        dynamicLawnmower.reset(new DynamicLawnmower(lawnmower3Start,
                                                    0,
                                                    M_PI * 3 / 2,
                                                    parameters.initalSpacing,
                                                    plumeHeight,
                                                    4,
                                                    0.5,
                                                    2));
    }
}

bool NestedBinVentPlanner::isGoalSurvey(double nestedBinSize, DataNode* maximum, std::vector<DataNode*>& neighbors)
{
    bool goalSurvey = false;
    if(nestedBinSize <= parameters.finalSpacing + 0.1)
    {
        DataNode& smallestCenter = dataTree->getSmallestNode(VehiclePose(0,0,0));

        if(maximum == &smallestCenter)
        {
            goalSurvey = true;
        }

        for(auto neighbor : neighbors)
        {
            if(&smallestCenter == neighbor)
            {
                goalSurvey = true;
            }
        }
    }

    return goalSurvey;
}