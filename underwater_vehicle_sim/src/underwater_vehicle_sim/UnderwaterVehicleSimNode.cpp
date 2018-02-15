#include "ros/ros.h"

#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

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
    

    UnderwaterVehicleSim sim(nh);

    //Run the simulation loop
    while(ros::ok())
    {
    	sim.update();

        ros::spinOnce();
        r.sleep();
    }
}