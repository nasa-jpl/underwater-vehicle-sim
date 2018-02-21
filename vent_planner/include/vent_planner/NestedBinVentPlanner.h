#ifndef NESTED_BIN_VENT_PLANNER_H
#define NESTED_BIN_VENT_PLANNER_H

#include <vector>
#include <memory>
#include <stack>
#include <functional>

#include "tf/LinearMath/Vector3.h"

#include "planner_framework/Planner.h"

#include "vent_planner/VentActionFactory.h"

#include "vent_planner/DataBin.h"
#include "vent_planner/DataBins.h"


#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeData.h"
#include "plume_detector/GetPlumeData.h"

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

    void plumeDataSummary(double& average, double& max, double& stddev);
    bool getHeightOfPlume(const std::vector<PlumeData>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeHeight, double& plumeStrength);
    void getPlumeMax(const std::vector<PlumeData>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeStrength);

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

    std::stack<std::shared_ptr<Plan>> plans;

    DataBin spiralData;
    std::unique_ptr<DataBins> dataBins;
    double plumeHeight;
    unsigned int currentInitalLawnmower;

    ros::Time lastPlan;
    bool initalPlan;

    double spiralSpacing;
    double initalSpacing;
    double finalSpacing;

    std::string vehicleName;
    tf::Vector3 vehicleStartLocation;

    ros::ServiceClient dataClient;
    ros::ServiceClient latestDataClient;
    ros::ServiceClient plumeClient;
    ros::NodeHandle& nh;
};

#endif