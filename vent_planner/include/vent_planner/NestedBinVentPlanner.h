#ifndef NESTED_BIN_VENT_PLANNER_H
#define NESTED_BIN_VENT_PLANNER_H

#include <vector>
#include <set>
#include <memory>
#include <stack>
#include <functional>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"

#include "vent_planner/actions/VentActionFactory.h"
#include "vent_planner/controllers/DynamicLawnmowerController.h"
#include "vent_planner/controllers/PointPathController.h"

#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"


#include "data_server/DataServerEntry.h"

#include "data_server/GetPlumeData.h"
#include "data_server/PlumeData.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"

class NestedBinVentPlanner : public Planner
{
public:
    NestedBinVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, VehicleInfo vehicleInfo);
    ~NestedBinVentPlanner() {}

    std::shared_ptr<Plan> plan();

    

    DataServerEntry getLatestData();

private:

    enum SearchPhase {none, spiral, dynamic, nested };

    void receivePlumeData(const data_server::PlumeData::ConstPtr& msg);

    void publishLog(std::string log);

    DataNode getLatestSpiralData();
    bool newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold);

    void initalizeDataTree(tf::Vector3 centerLocation);

    /**
    *Sets the parameter returnEntry to the latest data from the vehicle
    *@param returnEntry Output for the latest data
    *@return True if getting the latest data was successful
    **/
    bool getLatestData(DataServerEntry& returnEntry);
    std::set<DataNode*, DataNode::PointerCompare> getUnexploredMaxima();
    void addInitalLawnmowers(std::shared_ptr<Plan> plan, const tf::Vector3& centerLocation, double plumeHeight);


    bool isGoalSurvey(double nestedBinSize, DataNode* maximum, std::vector<DataNode*>& neighbors);
    void publishGoal();
    void updateGoal();

private:
    std::unique_ptr<VentActionFactory> actionFactory;

    SearchPhase phase;

    std::shared_ptr<Plan> spiralPlan;
    std::shared_ptr<Plan> dynamicPlan;

    std::unique_ptr<DataTree> dataTree;
    DataNode spiralData;

    ros::Subscriber dataSub;
    std::map<std::shared_ptr<Plan>, DataNode*> plannedMaxima;


    ros::Time lastPlan;

    double spiralSpacing;
    double initalSpacing;
    double finalSpacing;
    double failTime;
    std::shared_ptr<Plan> finalSurvey;

    VehicleInfo vehicleInfo;

    ros::ServiceClient dataClient;
    ros::ServiceClient latestDataClient;
    ros::ServiceClient plumeClient;
    ros::Publisher goalPub;
    ros::Publisher logPub;
    ros::NodeHandle& nh;

    std::string goalState;

    DynamicLawnmowerController dynamicLawnmowerController;
    PointPathController pointPathController;
};

#endif