#include "vehicle_auto_control/PropulsionController.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "vehicle_auto_control/VehicleController.h"
#include "vehicle_auto_control/FourDOFPropulsionLogic.h"

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

PropulsionController::PropulsionController(ros::NodeHandle& nh, VehicleInfo& info) :
	controlNode(nh, "vehicle_controller/" + info.getName()), 
	vehicleNode(nh, "underwater_vehicle_sim/vehicles/" + info.getName()),
	info(info),
	listener(buffer),
	logicController(PropulsionLogicInterface::makePropulsionLogic(info)),
	goToXYServer(controlNode, "go_to_xy", false),
	goToZServer(controlNode, "go_to_z", false)
{
	std::vector<std::string> dataModuleNames = info.getModuleNamesOfType("DataBroadcaster");
	
	if(dataModuleNames.size() > 0)
    {
		//default to using first module of type DataBroadcaster if more than 1 exists
        dataSub = vehicleNode.subscribe(dataModuleNames[0] + "/data", 1, &PropulsionController::getVehicleData, this);
    }

	velocityPub = vehicleNode.advertise<geometry_msgs::Twist>(info.getPropModuleName() + "/command_velocity", 1000);
    velocitySub = controlNode.subscribe("command_target_velocity", 1, &PropulsionController::getTargetVelocityCommand, this);

	goToXYServer.registerGoalCallback(boost::bind(&PropulsionController::goalGoToXYCB, this));
    goToXYServer.registerPreemptCallback(boost::bind(&PropulsionController::preemptGoToXYCB, this));
 	goToXYServer.start();

    goToZServer.registerGoalCallback(boost::bind(&PropulsionController::goalGoToZCB, this));
    goToZServer.registerPreemptCallback(boost::bind(&PropulsionController::preemptGoToZCB, this));
	goToZServer.start();
}

void PropulsionController::getTargetVelocityCommand(const geometry_msgs::Twist vel)
{
	logicController->setTargetVelocity(vel);
}

void PropulsionController::getVehicleData(const underwater_vehicle_msgs::VehicleData data)
{
	logicController->processNewData(data);
}

void PropulsionController::update(void)
{
	if(goToXYServer.isActive())
    {
        goToXYUpdate();
    }

    if(goToZServer.isActive())
    {
        goToZUpdate();
    }
}

/**
* Accepts new goals for the GoToXY SimpleActionServer
*/ 
void PropulsionController::goalGoToXYCB(void)
{
	velocityPub.publish(logicController->stopXYTwist());
    vehicle_auto_control::GoToXYRosGoalConstPtr goToXYGoal = goToXYServer.acceptNewGoal();
    
	logicController->setTargetXY(goToXYGoal->x, goToXYGoal->y);
    ROS_INFO("GoToXY server accepted a new goal - x:%f y:%f", goToXYGoal->x, goToXYGoal->y);
}

void PropulsionController::preemptGoToXYCB(void)
{
	velocityPub.publish(logicController->stopXYTwist());
	
    goToXYServer.setPreempted();
}

void PropulsionController::goToXYUpdate(void)
{
	tf2::Stamped<tf2::Transform> transform;
	try
	{
		transform = getCurrentTransform();
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	vehicle_auto_control::GoToXYRosFeedback feedback;
	feedback.x = transform.inverse().getOrigin().getX();
	feedback.y = transform.inverse().getOrigin().getY();
	goToXYServer.publishFeedback(feedback);

	if(logicController->isAtXY(transform))
	{
		velocityPub.publish(logicController->stopXYTwist());
		vehicle_auto_control::GoToXYRosResult result;
		result.x = transform.inverse().getOrigin().getX();
		result.y = transform.inverse().getOrigin().getY();
		goToXYServer.setSucceeded(result);
		ROS_INFO("GoToXY Server goal completed: %f %f", result.x, result.y);
	}
	else
	{
		velocityPub.publish(logicController->goToXYTwist(transform));

		//Call this if Z is not active to prevent the vehicle from hitting the seafloor
		if(!goToZServer.isActive())
		{
			velocityPub.publish(logicController->goToZTwist(transform));
		}
	}
}

/**
* Accepts new goals for the GoToZ SimpleActionServer
*/ 
void PropulsionController::goalGoToZCB(void)
{
	velocityPub.publish(logicController->stopZTwist());
    vehicle_auto_control::GoToZRosGoalConstPtr goToZGoal = goToZServer.acceptNewGoal();

	logicController->setTargetZ(goToZGoal->z);
    ROS_INFO("GoToZ server accepted a new goal - z: %f", goToZGoal->z);
}

void PropulsionController::preemptGoToZCB(void)
{
	velocityPub.publish(logicController->stopZTwist());
	
    goToZServer.setPreempted();
}

void PropulsionController::goToZUpdate(void)
{
	tf2::Stamped<tf2::Transform> transform;
	try
	{
		transform = getCurrentTransform();
	}
	catch(tf2::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	vehicle_auto_control::GoToZRosFeedback feedback;
    feedback.z = transform.getOrigin().getZ();
    goToZServer.publishFeedback(feedback);

	if(logicController->isAtZ(transform))
	{
		velocityPub.publish(logicController->stopZTwist());
		vehicle_auto_control::GoToZRosResult result;
        result.z = transform.getOrigin().getZ();
        goToZServer.setSucceeded(result); 
		ROS_INFO("GoToZ Server goal completeted: %f", result.z);
	}
	else
	{
		velocityPub.publish(logicController->goToZTwist(transform));
	}
}

tf2::Stamped<tf2::Transform> PropulsionController::getCurrentTransform()
{
	geometry_msgs::TransformStamped transformMsg;
    tf2::Stamped<tf2::Transform> transform;
	try
    {
        if(buffer.canTransform(info.getName(), "world_ned", ros::Time(0), ros::Duration(10.0)))
        {
            transformMsg = buffer.lookupTransform(info.getName(), "world_ned", ros::Time(0));
        }
		tf2::fromMsg(transformMsg, transform);
	}
	catch(tf2::TransformException ex)
	{
		throw ex;
	}

	return transform;
}