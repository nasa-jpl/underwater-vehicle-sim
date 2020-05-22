#ifndef FOUR_DOF_PROPULSION_PID_LOGIC_H
#define FOUR_DOF_PROPULSION_PID_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Vector3.h"

#include "propulsion_controller/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/VehicleData.h"

class FourDOFPropulsionPIDLogic : public PropulsionLogicInterface
{

public:
	FourDOFPropulsionPIDLogic(VehicleInfo& vehicleInfo);
	~FourDOFPropulsionPIDLogic() {}
	
	void goToXY(underwater_autonomy::VehiclePose& pose) override;
	void followHeading(underwater_autonomy::VehiclePose& pose) override;
	void goToZ(underwater_autonomy::VehiclePose& pose) override;
	void avoidSeafloor(underwater_autonomy::VehiclePose& pose) override;

	void stopXY() override;
	void stopZ() override;

	bool isAtXY(underwater_autonomy::VehiclePose& pose);
	bool isAtZ(underwater_autonomy::VehiclePose& pose);

	void processNewData(const underwater_vehicle_msgs::VehicleData data) override;

private:

	void forwardThrusterControlEffortCB(std_msgs::Float64 data);
	void lateralThrusterControlEffortCB(std_msgs::Float64 data);
	void verticalThrusterControlEffortCB(std_msgs::Float64 data);
	void rudderControlEffortCB(std_msgs::Float64 data);

	double scaleHorizontalVelocity(double distance);
	double scaleVerticalVelocity(double zDifference);

private:
	//Error bars for claiming the vehicle is at a point
	double lateralError;
	double verticalError;

	double latestSonarDepth;

	//Minimum distance off seafloor
	double minSeafloorDistance;

	//Error value for proportional controller
	double horizontalScaleError;
	double verticalErrorScale;

	tf2::Vector3 targetLinearVelocity;
	tf2::Vector3 targetAngularVelocity;

	geometry_msgs::Vector3 lastLinearVelocity;
    geometry_msgs::Vector3 lastAngularVelocity;

	bool xyEnabled;
	bool zEnabled;
	

	ros::Publisher forwardThrustPub;
	ros::Publisher lateralThrustPub;
	ros::Publisher verticalThrustPub;
	ros::Publisher rudderPub;

	//Forward Thruster Pub/Sub
	ros::Publisher forwardThrusterState;
	ros::Publisher forwardThrusterSetpoint;
	ros::Publisher forwardThrusterEnable;
	ros::Subscriber forwardThrusterControlEffort;

	//Lateral Thruster Pub/Sub
	ros::Publisher lateralThrusterState;
	ros::Publisher lateralThrusterSetpoint;
	ros::Publisher lateralThrusterEnable;
	ros::Subscriber lateralThrusterControlEffort;

	//Vertical Thruster Pub/Sub
	ros::Publisher verticalThrusterState;
	ros::Publisher verticalThrusterSetpoint;
	ros::Publisher verticalThrusterEnable;
	ros::Subscriber verticalThrusterControlEffort;

	//Rudder Pub/Sub
	ros::Publisher rudderState;
	ros::Publisher rudderSetpoint;
	ros::Publisher rudderEnable;
	ros::Subscriber rudderControlEffort;
};


#endif