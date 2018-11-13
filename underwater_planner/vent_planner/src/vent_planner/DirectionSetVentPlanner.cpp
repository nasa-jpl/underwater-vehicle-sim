#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "planner_framework/VehicleInterface.h"
#include "planner_framework/PlannerData.h"
#include "planner_framework/VehiclePose.h"
#include "planner_framework/GoalStatus.h"

#include "vent_planner/DataNode.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/DirectionSetVentPlanner.h"

#include "vent_planner/util/CreatePathUtil.h"
#include "vent_planner/util/MathUtil.h"
#include "vent_planner/util/Plane.h"

DirectionSetVentPlanner::DirectionSetVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters) :
    actionFactory(std::move(actionFactory)),
    vehicleInterface(std::move(vehicleInterface)),
    parameters(std::move(parameters)),
    currentPlannerStage(SearchPhase::INITIAL_PLAN),
    spiralData(nullptr, 0, VehiclePose(0,0,0), 300000, 0),
    receivingData(false)
{
    this->vehicleInterface->registerDataCallback(std::bind(&DirectionSetVentPlanner::receivePlumeData, this, std::placeholders::_1));
}

void DirectionSetVentPlanner::receivePlumeData(const PlannerData& data)
{
    if(currentPlannerStage == SearchPhase::SPIRAL)
    {
        spiralData.addData(data);
    }
    else
    {
        if(!std::isnan(data.getPose().getX()) &&
           !std::isnan(data.getPose().getY()) &&
           !std::isnan(data.getData()["plume"]))
        {
            currentData.push_back(data);

            if(data.getData()["plume"] > currentMax.getData()["plume"])
            {
                if(math_util::xyDistance(data.getPose(), currentMax.getPose()) > parameters.newMaxThreshold)
                {
                    currentMax = data;
                    numMaxCrossings = 1;
                }
            } 
        }
    }

    receivingData = true;
}

bool DirectionSetVentPlanner::endTransect()
{
    PlannerData& lastPoint = currentData[currentData.size() - 1];
    //Check to insure no violation of min and max leg lengths
    if(math_util::xyDistance(lastPoint.getPose(), currentLineCenter) < parameters.minLegLength)
    {
        return false;
    }
    else if(math_util::xyDistance(lastPoint.getPose(), currentLineCenter) >= parameters.maxLegLength)
    {
        return true;
    }
    

    int start = currentData.size() - 1;
    while(start >= 0 &&
          math_util::xyDistance(currentData[start].getPose(), lastPoint.getPose()) < parameters.legSectionLength * parameters.numSectionsThreshold)
    {
        start--;
    }
    
    //Don't end transect because we have not travelled far
    //enough to do all section calculations
    if(start == -1)
    {
        return false;
    }

    std::vector<double> average(parameters.numSectionsThreshold);
    std::vector<unsigned int> dataCount(parameters.numSectionsThreshold);

    PlannerData&  startPoint = currentData[start];
    for(unsigned int i = start; i < currentData.size() - 1; i++)
    {
        unsigned int section = math_util::xyDistance(currentData[i].getPose(), startPoint.getPose()) / parameters.legSectionLength;
        if(section < parameters.numSectionsThreshold)
        {
            average[section] += currentData[i].getPose().getZ();
            dataCount[section]++;
        }
    }

    double lastAverage = parameters.detectionThreshold;
    for(unsigned int i = 0; i < average.size(); i++)
    {
        average[i] /= dataCount[i];
        if(average[i] > lastAverage)
        {
            return false;
        }
        lastAverage = average[i];
    }
    
    return true;
}

std::shared_ptr<Plan> DirectionSetVentPlanner::plan()
{
    vehicleInterface->log(LogLevel::INFO, "Plan");

    std::shared_ptr<Plan> returnPlan(nullptr);


    if(currentPlannerStage == SearchPhase::INITIAL_PLAN)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase INITIAL_PLAN");
        if(receivingData)
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());

            VehiclePose pose = vehicleInterface->getPosition();

            std::vector<VehiclePose> spiralPoints = create_path_util::makeSpiral(pose, 0, parameters.spiralSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     -100,
                                                                                     -2000,
                                                                                     spiralPoints,
                                                                                     PointPathAction::ReplanType::ON_YOYO_TURN,
                                                                                     0);
            returnPlan->addAction(newAction);

            currentPlannerStage = SearchPhase::SPIRAL;
        }
        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::SPIRAL)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase SPIRAL");
        PlannerData maxVal = spiralData.getMaxVal();
        if(maxVal.getData()["plume"] >= parameters.detectionThreshold)
        {
            currentMax = maxVal;
         
            numMaxCrossings = 0;
            currentHeading = 0;

            plumeHeight = spiralData.getHeightOfPlume();

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::EXECUTE_LINE_0;     
        }

        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_0)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase EXECUTE_LINE_0");

        //Update heading and go to center of new transect
        returnPlan = std::shared_ptr<Plan>(new Plan());

        if(numMaxCrossings <= 1)
        {
            currentHeading += M_PI / 2;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            }
        }
        else if(numMaxCrossings == 2)
        {
            //Do line at 45 degrees
            currentHeading += M_PI / 4;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            } 
        }
        else if(numMaxCrossings == 3)
        {
            //Do other 45 degree line
            currentHeading += M_PI / 2;
            if(currentHeading >= M_PI * 2)
            {
                currentHeading -= M_PI * 2;
            } 
        }
        else
        {
            vehicleInterface->sendGoalStatus(GoalStatus::SUCCESS);
        }
        numMaxCrossings++;

        currentLineCenter = currentMax.getPose();
        std::vector<VehiclePose> points;
        VehiclePose p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        points.push_back(p1);
        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          points,
                                                                                          PointPathAction::ReplanType::NONE,
                                                                                          0);

        returnPlan->addAction(newAction);
        currentPlannerStage = SearchPhase::EXECUTE_LINE_1;
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_1)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase EXECUTE_LINE_1");

        //Go to first end of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());

        std::vector<VehiclePose> points;
        VehiclePose pose1(currentLineCenter.getX(), 
                          currentLineCenter.getY(), 
                          plumeHeight);

        VehiclePose pose2(currentLineCenter.getX() + (sin(currentHeading) * parameters.maxLegLength), 
                          currentLineCenter.getY() + (cos(currentHeading) * parameters.maxLegLength), 
                          plumeHeight);

        points.push_back(pose1);
        points.push_back(pose2);

        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          points,
                                                                                          PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                          parameters.legSectionLength);

        returnPlan->addAction(newAction);
        transectPlan = returnPlan;
        currentData.clear();

        currentPlannerStage = SearchPhase::OBSERVE_EXECUTE_LINE_1; 
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_EXECUTE_LINE_1)
    {
        if(transectPlan->isCompleted() || endTransect())
        {
            //Send back empty plan to trigger replan again (not an ideal way to do this)
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::EXECUTE_LINE_2;
        }
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_2)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase EXECUTE_LINE_2");

        //Go to center of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());
        std::vector<VehiclePose> points;
        VehiclePose p1(currentLineCenter.getX(), currentLineCenter.getY(), plumeHeight);
        points.push_back(p1);
        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          points,
                                                                                          PointPathAction::ReplanType::NONE,
                                                                                          0);

        returnPlan->addAction(newAction);
        currentPlannerStage = SearchPhase::EXECUTE_LINE_3;
    }
    else if(currentPlannerStage == SearchPhase::EXECUTE_LINE_3)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase EXECUTE_LINE_3");

        //Go to second end of transect
        returnPlan = std::shared_ptr<Plan>(new Plan());

        std::vector<VehiclePose> points;
        VehiclePose pose1(currentLineCenter.getX(), 
                          currentLineCenter.getY(), 
                          plumeHeight);

        VehiclePose pose3(currentLineCenter.getX() + (sin(currentHeading + M_PI) * parameters.maxLegLength), 
                          currentLineCenter.getY() + (cos(currentHeading + M_PI) * parameters.maxLegLength), 
                          plumeHeight);

        points.push_back(pose1);
        points.push_back(pose3);

        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          points,
                                                                                          PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                          parameters.legSectionLength);

        returnPlan->addAction(newAction);
        transectPlan = returnPlan;
        currentData.clear();

        currentPlannerStage = SearchPhase::OBSERVE_EXECUTE_LINE_3;
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_EXECUTE_LINE_3)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase OBSERVE_EXECUTE_LINE_3");

        if(transectPlan->isCompleted() || endTransect())
        {
            //Send back empty plan to trigger replan again  (not an ideal way to do this)
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::EXECUTE_LINE_0;
        }
    }

    return returnPlan;
}