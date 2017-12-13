#include "ros/ros.h"

#include "underwater_vehicle_sim/UnderwaterVehicleSim.h"
#include "vehicles/Vehicle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");
    ros::NodeHandle n;

    ros::Rate r(100); //Hz at which to run the sim loop

    UnderwaterVehicleSim sim;

    //Run the simulation loop
    while(ros::ok())
    {
    	sim.update();

        ros::spinOnce();
        r.sleep();
    }
}