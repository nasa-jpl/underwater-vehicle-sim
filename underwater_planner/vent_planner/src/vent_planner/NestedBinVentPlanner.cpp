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
    phase(SearchPhase::none),
    spiralData(nullptr, 0, VehiclePose(0,0,0), 300000, 0),
    receivingData(false)
{
    this->vehicleInterface->registerDataCallback(std::bind(&NestedBinVentPlanner::receivePlumeData, this, std::placeholders::_1));
}

void NestedBinVentPlanner::receivePlumeData(const PlannerData& data)
{
    if((phase == SearchPhase::dynamic || phase == SearchPhase::nested) &&
       dataTree)
    {
        dataTree->addData(data);
    }
    else if(phase == SearchPhase::spiral)
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

    if(phase == SearchPhase::none)
    {
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

            phase = SearchPhase::spiral;
        }
    }
    else if(phase == SearchPhase::spiral)
    {
        bool valid = newSpiralPlumeIntersect(spiralData, detectionThreshold);

        if(valid)
        {
            PlannerData maxVal = spiralData.getMaxVal();
            double plumeHeight = spiralData.getHeightOfPlume();

            initalizeDataTree(maxVal.getPose());

            returnPlan = std::shared_ptr<Plan>(new Plan());
            dynamicPlan = returnPlan;
            phase = SearchPhase::dynamic;

            const VehiclePose closestOrigin = dataTree->getClosestNodeOrigin(maxVal.getPose(), 1);
            addInitalLawnmowers(returnPlan, closestOrigin, plumeHeight);

            //Log data to file
            std::stringstream ss;
            ss.precision(5);
            ss << std::fixed << "DynamicLawnmower," << plumeHeight << "," << closestOrigin.getX() << "," << closestOrigin.getY() << "," << parameters.initalSpacing;
            vehicleInterface->log(LogLevel::INFO, ss.str());
        }
        else
        {
            returnPlan = spiralPlan;
            returnPlan->resetInterrupted();
            phase = SearchPhase::spiral;
        }

        spiralData.clear();
    }
    else if(phase == SearchPhase::dynamic || phase == SearchPhase::nested)
    {
        std::set<DataNode*, DataNode::PointerCompare> queuedMaxima = getUnexploredMaxima();

        if(queuedMaxima.size() > 0)
        {
            auto lastElement = queuedMaxima.end();
            --lastElement;
            DataNode* maximum = *lastElement;

            double nestedBinSize = maximum->getSize() / nestedSizeFactor;

            std::vector<DataNode*> neighbors = maximum->getInitalizedNeighbors();

            //Check for goal completion.
            //This should be moved to a seperate function at some point

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
            phase = SearchPhase::nested;

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
            if(!dynamicPlan->isCompleted())
            {
                returnPlan = dynamicPlan;
                phase = SearchPhase::dynamic;
            }
            else
            {
                returnPlan = spiralPlan;
                phase = SearchPhase::spiral;
            }
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

void NestedBinVentPlanner::addInitalLawnmowers(std::shared_ptr<Plan> plan, const VehiclePose& centerLocation, double plumeHeight)
{
    VehiclePose lawnmower0Start(centerLocation.getX() + parameters.initalSpacing / 2.0, centerLocation.getY() + parameters.initalSpacing / 2.0, plumeHeight);
    VehiclePose lawnmower1Start(centerLocation.getX() - parameters.initalSpacing / 2.0, centerLocation.getY() + parameters.initalSpacing / 2.0, plumeHeight);
    VehiclePose lawnmower2Start(centerLocation.getX() - parameters.initalSpacing / 2.0, centerLocation.getY() - parameters.initalSpacing / 2.0, plumeHeight);
    VehiclePose lawnmower3Start(centerLocation.getX() + parameters.initalSpacing / 2.0, centerLocation.getY() - parameters.initalSpacing / 2.0, plumeHeight);

    std::shared_ptr<Action> lawnmower0 = actionFactory->createDynamicLawnmowerAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower0Start,
                                                                                     0,
                                                                                     M_PI / 2,
                                                                                     parameters.initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower1 = actionFactory->createDynamicLawnmowerAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower1Start,
                                                                                     M_PI,
                                                                                     M_PI / 2,
                                                                                     parameters.initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower2 = actionFactory->createDynamicLawnmowerAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower2Start,
                                                                                     M_PI,
                                                                                     M_PI * 3 / 2,
                                                                                     parameters.initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);

    std::shared_ptr<Action> lawnmower3 = actionFactory->createDynamicLawnmowerAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     lawnmower3Start,
                                                                                     0,
                                                                                     M_PI * 3 / 2,
                                                                                     parameters.initalSpacing,
                                                                                     plumeHeight,
                                                                                     4,
                                                                                     0.5,
                                                                                     2);
                                                                                     

    plan->addAction(lawnmower0);
    plan->addAction(lawnmower1);
    plan->addAction(lawnmower2);
    plan->addAction(lawnmower3);
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
