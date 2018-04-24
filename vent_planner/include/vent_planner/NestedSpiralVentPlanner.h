#ifndef VENT_PLANNER_H
#define VENT_PLANNER_H

#include <vector>
#include <memory>
#include <stack>
#include <functional>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"
#include "vent_planner/VentActionFactory.h"

#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeDataEntry.h"
#include "data_server/GetPlumeData.h"

class NestedSpiralVentPlanner : public Planner
{
public:
    NestedSpiralVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName);
    ~NestedSpiralVentPlanner() {}

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

    void plumeDataSummary(double& average, double& max, double& stddev);
    bool triggerNewSpiral(const double plumeStrength);
    bool getHeightOfPlume(const std::vector<PlumeDataEntry>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeHeight, double& plumeStrength);
    void getPlumeMax(const std::vector<PlumeDataEntry>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeStrength);

    bool isCompleted(std::shared_ptr<Plan> plan);

    bool isDone();
    /**
    *Sets the parameter returnEntry to the latest data from the vehicle
    *@param returnEntry Output for the latest data
    *@return True if getting the latest data was successful
    **/
    bool getLatestData(DataServerEntry& returnEntry);

private:
    std::unique_ptr<VentActionFactory> actionFactory;

    std::vector<std::vector<PlumeDataEntry>> plumeData;

    std::stack<std::shared_ptr<Plan>> plans;
    std::stack<unsigned long> currentPlumeData;
    double plumeHeight;

    ros::Time lastPlan;
    bool initalPlan;

    double initalSpacing;
    double finalSpacing;
    double triggerSigma;

    unsigned int yoyoUpperDepth;
    unsigned int yoyoLowerDepth;
    std::string vehicleName;
    tf::Vector3 vehicleStartLocation;

    ros::ServiceClient dataClient;
    ros::ServiceClient latestDataClient;

    ros::ServiceClient plumeClient;

    ros::NodeHandle& nh;
};

#endif