#include "ros/ros.h"

#include "vehicles/Vehicle.h"
#include "std_msgs/Float64.h"

std::unique_ptr<Vehicle> vehicle;

bool getVehicleInfo(underwater_vehicle_msgs::GetVehicleInfo::Request &req,
				  	underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
    vehicle->getInfo(res);
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");

    vehicle.reset(new Vehicle());

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    double hertz;
    if(!nhPriv.getParam("sim_hertz", hertz))
    {
        ROS_FATAL("Parameter \"%s/sim_hertz\" not present in the parameter server.", nhPriv.getNamespace().c_str());
        exit(1);
    }

    nhPriv.getParam("sim_hertz", hertz);
    ros::Rate r(hertz); //Hz at which to run the sim loop
    
    ros::Publisher clockSpeedPub = nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1, true);
    std_msgs::Float64 slowSim;
    slowSim.data = 1;
    clockSpeedPub.publish(slowSim);


    ros::ServiceClient modelDataClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");

    //Wait for model
    if(nhPriv.hasParam("/model_type"))
    {
        modelDataClient.waitForExistence();
    }

    ros::ServiceServer service = nh.advertiseService("get_info", &getVehicleInfo);;

    //Run the simulation loop
    while(ros::ok())
    {
    	vehicle->update();

        ros::spinOnce();
        r.sleep();
    }
}