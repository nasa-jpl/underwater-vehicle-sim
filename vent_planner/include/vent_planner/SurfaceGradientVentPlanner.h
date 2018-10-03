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

    std::shared_ptr<Plan> plan();

    /**
    *Fits plane to points using least squares minimizing distance in z direction.
    *Output in format h = a0x + a1y + b.
    *@param points Input points
    *@param a0 Output scalar
    *@param a1 Output scalar
    *@param b Output scalar
    */
    static bool fitPlane(std::vector<tf::Vector3>& points, double& a0, double& a1, double& b);

    static double planeGradientHeading(const double a0, const double a1);

private:

    void publishGoal();
    void updateGoal();



private:

    enum SearchPhase { INITIAL_PLAN, CALC_GRADIENT, FOLLOW_GRADIENT };

    std::unique_ptr<VentActionFactory> actionFactory;  
    std::string goalState;
    std::string vehicleName;
    ros::NodeHandle& nh;

    SearchPhase currentPlannerStage;

    double failTime;
    ros::Publisher goalPub;
};

#endif