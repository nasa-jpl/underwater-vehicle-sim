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

#include "plume_detector/PlumeData.h"
#include "plume_detector/GetPlumeData.h"

class VentPlanner : public Planner
{
public:
	VentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName);
	~VentPlanner() {}

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
	bool triggerNewSpiral(const double plumeHeight, const double plumeStrength);
	bool getHeightOfPlume(const std::vector<PlumeData>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeHeight, double& plumeStrength);

	bool isCompleted(std::shared_ptr<Plan> plan);

	/**
	*Sets the parameter returnEntry to the latest data from the vehicle
	*@param returnEntry Output for the latest data
	*@return True if getting the latest data was successful
	**/
	bool getLatestData(DataServerEntry& returnEntry);

private:
	std::unique_ptr<VentActionFactory> actionFactory;

	std::vector<std::vector<PlumeData>> plumeData;

	std::stack<std::shared_ptr<Plan>> plans;
	std::stack<unsigned long> currentPlumeData;

	ros::Time lastPlan;
	bool initalPlan;

	double initalSpacing;

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