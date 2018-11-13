#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/SurfaceGradientVentPlanner.h"
#include "vent_planner/DataNode.h"
#include "vent_planner/util/CreatePathUtil.h"
#include "vent_planner/util/MathUtil.h"
#include "vent_planner/util/Plane.h"

SurfaceGradientVentPlanner::SurfaceGradientVentPlanner(std::unique_ptr<VentActionFactory> actionFactory, std::unique_ptr<VehicleInterface> vehicleInterface, Parameters parameters) :
    actionFactory(std::move(actionFactory)),
    parameters(std::move(parameters)),
    vehicleInterface(std::move(vehicleInterface)),
    currentPlannerStage(SearchPhase::INITIAL_PLAN),
    spiralData(nullptr, 0, VehiclePose(0,0,0), 300000, 0),
    receivingData(false)
{
    this->vehicleInterface->registerDataCallback(std::bind(&SurfaceGradientVentPlanner::receivePlumeData, this, std::placeholders::_1));
}

void SurfaceGradientVentPlanner::receivePlumeData(const PlannerData& data)
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
        }

    }
    receivingData = true;
}

bool SurfaceGradientVentPlanner::endGradientFollow()
{
    if(currentData.size() == 0)
    {
        return false;
    }

    PlannerData& lastPoint = currentData[currentData.size() - 1];
    //Check that we are at least a window size past the initial point on the circle's edge
    if(gradientFollowAction->getCurrentPoint() >= 1 &&
        math_util::xyDistance(lastPoint.getPose(), gradientFollowAction->getPoints()[0]) >= parameters.gradientWindow &&
        math_util::xyDistance(lastPoint.getPose(), gradientFollowAction->getPoints()[0]) >= parameters.gradientMinFollowDistance)
    {
        
        //Find start of data to use for follow gradient calculation
        unsigned int start = currentData.size() - 1;
        while(start >= 0)
        {
            if(!std::isnan(currentData[start].getPose().getX()) &&
               !std::isnan(currentData[start].getPose().getY()))
            {
                if(math_util::xyDistance(currentData[start].getPose(), lastPoint.getPose()) >= parameters.gradientWindow)
                {
                    break;
                }
            }
            start--;
        }

        if(start < 0)
        {
            start = 0;
        }

        std::vector<double> x;
        std::vector<double> y;
        double slope = 0.0;
        double yIntercept = 0.0;

        //Caluclate x and y values for follow graidnet calculation
        for(unsigned int i = start; i < currentData.size(); i++)
        {
            if(!std::isnan(currentData[i].getPose().getX()) &&
               !std::isnan(currentData[i].getPose().getY()))
            {
                x.push_back(math_util::xyDistance(currentData[start].getPose(), currentData[i].getPose()));
                y.push_back(currentData[i].getPose().getZ());
            }
        }
        
        math_util::linearLeastSquares(x, y, slope, yIntercept);

        return slope < parameters.gradientThreshold;
    }

    return false;
}

std::shared_ptr<Plan> SurfaceGradientVentPlanner::plan()
{
    vehicleInterface->log(LogLevel::INFO, "Plan");

    std::shared_ptr<Plan> returnPlan(nullptr);


    if(currentPlannerStage == SearchPhase::INITIAL_PLAN)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase INITIAL_PLAN");
        if(receivingData)
        {
            VehiclePose pose = vehicleInterface->getPosition();

            returnPlan = std::shared_ptr<Plan>(new Plan());

            std::vector<VehiclePose> spiralPoints = create_path_util::makeSpiral(pose, 0, parameters.spiralSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                                     0.49066,
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
            plumeHeight = spiralData.getHeightOfPlume();

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::PLAN_GRADIENT;     
        }

        spiralData.clear();
    }
    else if(currentPlannerStage == SearchPhase::PLAN_GRADIENT)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase PLAN_GRADIENT");
        currentData.clear();
        if(receivingData)
        {
            VehiclePose pose = vehicleInterface->getPosition();
            returnPlan = std::shared_ptr<Plan>(new Plan());
            gradientCirclePlan = returnPlan;

            gradientLocation.setX(pose.getX());
            gradientLocation.setY(pose.getY());
            gradientLocation.setZ(plumeHeight);

            std::vector<VehiclePose> gradientPoints = create_path_util::makePolygon(gradientLocation,
                                                   8, //sides
                                                   parameters.gradientCalcRadius,
                                                   0, //initial heading
                                                   true); //clockwise

            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(1.0,
                                                                                     0.349066,
                                                                                     0.523599, //30 deg
                                                                                     gradientPoints,
                                                                                     PointPathAction::ReplanType::NONE,
                                                                                     0);
            returnPlan->addAction(newAction);

            currentPlannerStage = SearchPhase::CALC_GRADIENT;
        }
    }
    else if(currentPlannerStage == SearchPhase::CALC_GRADIENT)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase CALC_GRADIENT");
        if(gradientCirclePlan->isCompleted())
        {
            Plane fitPlane = Plane::fitPlaneToPoints(currentData);
            gradientDirection = fitPlane.getHeightGradientHeading();

            //flip direction as gradient points downhill, but
            //we want to go up
            gradientDirection += M_PI; 

            //return empty plan to go to next phase
            returnPlan = std::shared_ptr<Plan>(new Plan()); 
            currentPlannerStage = SearchPhase::FOLLOW_GRADIENT;
        }
    }
    else if(currentPlannerStage == SearchPhase::FOLLOW_GRADIENT)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase FOLLOW_GRADIENT");
        returnPlan = std::shared_ptr<Plan>(new Plan());

        //Save the plan so we can check if it is complete later
        gradientFollowPlan = returnPlan;

        VehiclePose pose1(gradientLocation.getX() + (sin(gradientDirection) * parameters.gradientCalcRadius), 
                          gradientLocation.getY()  + (cos(gradientDirection) * parameters.gradientCalcRadius), 
                          gradientLocation.getZ());

        VehiclePose pose2(pose1.getX() + (sin(gradientDirection) * parameters.gradientMaxFollowDistance), 
                          pose1.getY() + (cos(gradientDirection) * parameters.gradientMaxFollowDistance), 
                          pose1.getZ());

        std::vector<VehiclePose> linePoints;
        linePoints.push_back(pose1);
        linePoints.push_back(pose2);

        std::shared_ptr<PointPathAction> newAction = actionFactory->createPointPathAction(1.0,
                                                                                          0.349066,
                                                                                          0.523599, //30 deg
                                                                                          linePoints,
                                                                                          PointPathAction::ReplanType::PERIODIC_DISTANCE,
                                                                                          200);

         //Save this action in a shared ptr so we can check how much we have completed later
        gradientFollowAction = newAction;

        returnPlan->addAction(newAction);

        //Move to observing the single dimensional gradient
        currentPlannerStage = SearchPhase::OBSERVE_FOLLOW_GRADIENT;
    }
    else if(currentPlannerStage == SearchPhase::OBSERVE_FOLLOW_GRADIENT)
    {
        vehicleInterface->log(LogLevel::INFO, "Phase OBSERVE_FOLLOW_GRADIENT");
        if(endGradientFollow() ||
           gradientFollowPlan->isCompleted())
        {
            returnPlan = std::shared_ptr<Plan>(new Plan());
            currentPlannerStage = SearchPhase::PLAN_GRADIENT;
        }
    }

    return returnPlan;
}