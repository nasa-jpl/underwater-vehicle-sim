#ifndef FOUR_DOF_PROPULSION_LOGIC_H
#define FOUR_DOF_PROPULSION_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_listener.h"

#include "actionlib/server/simple_action_server.h"

#include "vehicle_auto_control/PropulsionController.h"
#include "vehicle_auto_control/Velocity.h"
#include "vehicle_auto_control/GoToXYRosAction.h"
#include "vehicle_auto_control/GoToZRosAction.h"

#include "underwater_vehicle_msgs/VehicleData.h"

class FourDOFPropulsionLogic : public PropulsionLogicInterface
{

public:
	FourDOFPropulsionLogic(VehicleInfo& vehicleInfo);
	~FourDOFPropulsionLogic() {}
	
	const void goToXY(underwater_autonomy::VehiclePose& pose) override;
	const void goToZ(underwater_autonomy::VehiclePose& pose) override;
	
	const void stopXY() override;
	const void stopZ() override;

	void setTargetXY(double x, double y) override;
	void setTargetZ(double z) override;

	bool isAtXY(underwater_autonomy::VehiclePose& pose) override;
	bool isAtZ(underwater_autonomy::VehiclePose& pose) override;

	void setTargetVelocity(const geometry_msgs::Twist vel) override;
	void processNewData(const underwater_vehicle_msgs::VehicleData data) override;

private:

	void forwardThrusterControlEffortCB(std_msgs::Float64 data);
	void lateralThrusterControlEffortCB(std_msgs::Float64 data);
	void verticalThrusterControlEffortCB(std_msgs::Float64 data);
	void rudderControlEffortCB(std_msgs::Float64 data);

	double scaleHorizontalVelocity(double distance);
	double scaleVerticalVelocity(double zDifference);

private:
	double targetX;
	double targetY;
	double targetZ;

	double latestSonarDepth;
	double latestVehicleDepth;

	//Error bars for claiming the vehicle is at a point
	double lateralError;
	double verticalError;

	//Minimum distance off seafloor
	double minSeafloorDistance;
	bool hasVehicleData;

	//Error value for proportional controller
	double angleErrorScale;
	double verticalErrorScale;
	double horizontalScaleError;

	
	//Last command velocities
	geometry_msgs::Vector3 lastLinearVelocity;
    geometry_msgs::Vector3 lastAngularVelocity;

	tf2::Vector3 targetLinearVelocity;
	tf2::Vector3 targetAngularVelocity;

	double lastForwardThrust;
	double lastRudder;
	double lastVertThrust;

	ros::Publisher forwardThrustPub;
	ros::Publisher lateralThrustPub;
	ros::Publisher verticalThrustPub;
	ros::Publisher rudderPub;

	bool xyEnabled;
	bool zEnabled;
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