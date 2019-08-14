#ifndef UTH_PROPULSION_LOGIC_H
#define UTH_PROPULSION_LOGIC_H

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Vector3.h"

#include "vehicle_auto_control/PropulsionLogicInterface.h"

#include "underwater_vehicle_msgs/VehicleData.h"

class UthPropulsionLogic : public PropulsionLogicInterface
{

public:
	UthPropulsionLogic(VehicleInfo& vehicleInfo);
	~UthPropulsionLogic() {}
	
	void goToXY(underwater_autonomy::VehiclePose& pose) override;
	void followHeading(underwater_autonomy::VehiclePose& pose) override;
	void goToZ(underwater_autonomy::VehiclePose& pose) override;
	void avoidSeafloor(underwater_autonomy::VehiclePose& pose) override;

	void stopXY() override;
	void stopZ() override;

	void setTargetXY(double x, double y) override;
	void setTargetZ(double z) override;
	void setFollowHeading(double heading) override;

	bool isAtXY(underwater_autonomy::VehiclePose& pose) override;
	bool isAtZ(underwater_autonomy::VehiclePose& pose) override;

	void setTargetVelocity(const geometry_msgs::Twist vel) override;
	void processNewData(const underwater_vehicle_msgs::VehicleData data) override;

	bool getXYMovement();
	bool getZMovement();

	int getAvoidSeafloorCalls();
	int getGoToXYCalls();
	int getGoToZCalls();
	int getFollowHeadingCalls();
	int getStopXYCalls();
	int getStopZCalls();

	double getTargetX();
	double getTargetY();
	double getTargetZ();
	double getFollowHeading();

	void setAtXY(bool atXY);
	void setAtZ(bool atZ);

	geometry_msgs::Twist getTargetVelocity();

	underwater_vehicle_msgs::VehicleData getNewData();

private:

	void forwardThrusterControlEffortCB(std_msgs::Float64 data);
	void lateralThrusterControlEffortCB(std_msgs::Float64 data);
	void verticalThrusterControlEffortCB(std_msgs::Float64 data);
	void rudderControlEffortCB(std_msgs::Float64 data);

	double scaleHorizontalVelocity(double distance);
	double scaleVerticalVelocity(double zDifference);

private:
	bool xyMovement;
	bool zMovement;

	int avoidSeafloorCalls;
	int goToXYCalls;
	int goToZCalls;
	int followHeadingCalls;
	int stopXYCalls;
	int stopZCalls;

	double targetX;
	double targetY;
	double targetZ;
	double targetFollowHeading;

	bool atXY;
	bool atZ;

	geometry_msgs::Twist targetVelocity;

	underwater_vehicle_msgs::VehicleData newData;
};


#endif