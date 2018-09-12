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

#include "vent_planner/DataNode.h"
#include "vent_planner/DataTree.h"


#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeDataEntry.h"
#include "data_server/GetPlumeData.h"
#include "data_server/PlumeData.h"

class NestedBinVentPlanner : public Planner
{
public:
    NestedBinVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName);
    ~NestedBinVentPlanner() {}

    std::shared_ptr<Plan> plan();

    

    DataServerEntry getLatestData();

    static std::vector<tf::Vector3> makeSpiral(tf::Vector3 startLocation, 
                                        double startDirection, 
                                        double spacing, 
                                        double size);

    static std::vector<tf::Vector3> makeLawnmower(const tf::Vector3& startLocation,
                                                  double alongTrackDirection,
                                                  double acrossTrackDirection,
                                                  double alongTrackSize,
                                                  double acrossTrackSize,
                                                  double spacing);

private:

    enum SearchPhase {none, spiral, dynamic, nested };

    void receivePlumeData(const data_server::PlumeData::ConstPtr& msg);

    void publishLog(std::string log);

    bool isCompleted(std::shared_ptr<Plan> plan);

    DataNode getLatestSpiralData();
    bool newSpiralPlumeIntersect(DataNode& spiralData, double detectionThreshold);

    void initalizeDataTree(tf::Vector3 centerLocation);
    void addRecentDataToTree();
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

    std::string vehicleName;

    ros::ServiceClient dataClient;
    ros::ServiceClient latestDataClient;
    ros::ServiceClient plumeClient;
    ros::Publisher goalPub;
    ros::Publisher logPub;
    ros::NodeHandle& nh;

    std::string goalState;

    DynamicLawnmowerController dynamicLawnmowerController;
};

#endif