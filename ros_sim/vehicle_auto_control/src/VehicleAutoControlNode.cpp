#include "ros/ros.h"

#include "vehicle_auto_control/PropulsionController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_auto_control");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    float loopHertz;
    if(!nhPriv.getParam("hertz", loopHertz))
    {
        ROS_FATAL("Parameter \"%s/hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    std::unique_ptr<PropulsionController> controller;

    ros::ServiceClient infoClient = nh.serviceClient<underwater_vehicle_msgs::GetVehicleInfo>("get_info");
	infoClient.waitForExistence();

	underwater_vehicle_msgs::GetVehicleInfo info;
	infoClient.call(info);
	VehicleInfo vehicleInfo(info);

	if(vehicleInfo.getPropModuleType() != "")
	{
		controller.reset(new PropulsionController(vehicleInfo));
	}

    ros::Rate loop(loopHertz);
    while(ros::ok())
    {
        controller->update();
        ros::spinOnce();
        loop.sleep();
    }
        return 0;
}