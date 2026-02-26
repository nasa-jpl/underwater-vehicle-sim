#ifndef FOUR_DOF_PROPULSION_LOGIC_H
#define FOUR_DOF_PROPULSION_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Vector3.h"

#include "propulsion_controller/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#include "ros_underwater_sim_utilities/LinearPiecewise.h"
#include "ros_underwater_sim_utilities/VehiclePose.h"

class FourDOFPropulsionLogic : public PropulsionLogicInterface
{

public:
	FourDOFPropulsionLogic(VehicleInfo& vehicleInfo,
						   LinearPiecewise forwardThruster,
						   LinearPiecewise verticalThruster,
						   LinearPiecewise rudder);
	~FourDOFPropulsionLogic() {}
	
	void goToXY(VehiclePose& pose) override;
	void followHeading(VehiclePose& pose) override;
	void goToZ(VehiclePose& pose) override;
	void avoidSeafloor(VehiclePose& pose) override;

	void stopXY() override;
	void stopZ() override;

	bool isAtXY(VehiclePose& pose);
	bool isAtZ(VehiclePose& pose);

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
	LinearPiecewise forwardThruster;
	LinearPiecewise verticalThruster;
	LinearPiecewise rudder;

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