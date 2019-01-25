#ifndef FOUR_DOF_PROPULSION_LOGIC_H
#define FOUR_DOF_PROPULSION_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

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
	FourDOFPropulsionLogic(ros::NodeHandle vehicleNode, VehicleInfo& vehicleInfo);
	~FourDOFPropulsionLogic() {}
	
	const void goToXY(tf2::Stamped<tf2::Transform>& NEDToVehicle) override;
	const void goToZ(tf2::Stamped<tf2::Transform>& NEDToVehicle) override;
	
	const void stopXY() override;
	const void stopZ() override;

	void setTargetXY(double x, double y) override;
	void setTargetZ(double z) override;

	bool isAtXY(tf2::Stamped<tf2::Transform>& transform) override;
	bool isAtZ(tf2::Stamped<tf2::Transform>& transform) override;

	void setTargetVelocity(const geometry_msgs::Twist vel) override;
	void processNewData(const underwater_vehicle_msgs::VehicleData data) override;

private:

	double scaleHorizontalVelocity(double distance);
	double scaleVerticalVelocity(double zDifference);
	double scaleRotationalVelocity(double angle);

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

	//Publishers for messages to the FourDOFPropulsion module
	ros::Publisher velocityPub;

	//Last command velocities
	geometry_msgs::Vector3 lastLinearVelocity;
    geometry_msgs::Vector3 lastAngularVelocity;

	tf2::Vector3 targetLinearVelocity;
	tf2::Vector3 targetAngularVelocity;
};


#endif