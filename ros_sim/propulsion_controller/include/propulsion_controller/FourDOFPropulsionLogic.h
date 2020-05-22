#ifndef FOUR_DOF_PROPULSION_LOGIC_H
#define FOUR_DOF_PROPULSION_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Vector3.h"

#include "propulsion_controller/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#include "underwater_autonomy/util/LinearPiecewise.h"

class FourDOFPropulsionLogic : public PropulsionLogicInterface
{

public:
	FourDOFPropulsionLogic(VehicleInfo& vehicleInfo,
						   underwater_autonomy::LinearPiecewise forwardThruster,
						   underwater_autonomy::LinearPiecewise verticalThruster,
						   underwater_autonomy::LinearPiecewise rudder);
	~FourDOFPropulsionLogic() {}
	
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
	double scaleRotationalVelocity(double angle);

private:
	underwater_autonomy::LinearPiecewise forwardThruster;
	underwater_autonomy::LinearPiecewise verticalThruster;
	underwater_autonomy::LinearPiecewise rudder;

	//Error bars for claiming the vehicle is at a point
	double lateralError;
	double verticalError;

	double latestSonarDepth;

	//Minimum distance off seafloor
	double minSeafloorDistance;

	//Error value for proportional controller
	double horizontalErrorScale;
	double verticalErrorScale;
	double angleErrorScale;

	geometry_msgs::Vector3 lastLinearVelocity;
    geometry_msgs::Vector3 lastAngularVelocity;

	ros::Publisher forwardThrustPub;
	ros::Publisher lateralThrustPub;
	ros::Publisher verticalThrustPub;
	ros::Publisher rudderPub;


};


#endif