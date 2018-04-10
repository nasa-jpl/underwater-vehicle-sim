#include "ros/ros.h"

#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");
    ros::NodeHandle nh;

    double hertz;
    if(!nh.getParam("underwater_vehicle_sim/hertz", hertz))
    {
        ROS_FATAL("Parameter \"underwater_vehicle_sim/hertz\" not present in the parameter server.");
        exit(1);
    }
    
    nh.getParam("underwater_vehicle_sim/hertz", hertz);

    ros::Rate r(hertz); //Hz at which to run the sim loop
    
    ros::Publisher clockSpeedPub = nh.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1, true);
    std_msgs::Float64 stopSim;
    stopSim.data = 0;
    clockSpeedPub.publish(stopSim);

    UnderwaterVehicleSim sim(nh);

    ros::ServiceClient modelDataClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");

    //Wait for model
    if(nh.hasParam("model_type"))
    {
        modelDataClient.waitForExistence();
    }

    float speedUpFactor;
    nh.param<float>("speed_up_factor", speedUpFactor, 1);

    std_msgs::Float64 startSim;
    startSim.data = speedUpFactor;
    clockSpeedPub.publish(startSim);

    //Run the simulation loop
    while(ros::ok())
    {
    	sim.update();

        ros::spinOnce();
        r.sleep();
    }
}