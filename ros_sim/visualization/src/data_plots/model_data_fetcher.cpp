#include "ocean_models/model_interface/ModelData.h"

#include "ocean_models/model_interface/ModelInterface.h"
#include "ocean_models/fvcom/FVCOM.h"

#include "underwater_vehicle_msgs/VehicleData.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>


void startModelLoad()
{
    ;
    //std_msgs::Float64 slowSim;
    //slowSim.data = 1;
    //clockSpeedPub.publish(slowSim);
}

void endModelLoad()
{
    ;
    //std_msgs::Float64 startSim;
    //startSim.data = speedUpFactor;
    //clockSpeedPub.publish(startSim);
}


int main()
{
    // need to read in bag given by user
    rosbag::Bag bag;
    bag.open("/home/dev/Desktop/Parallels\ Shared\ Folders/Home/Documents/ocean_worlds/ros_workspace/vehicleData_2019-11-01-12-49-44.bag");  // BagMode is Read by default

    int lower_limit = 2000;
    lower_limit = 200;
    int upper_limit = 100;
    std::string fileName = "out.csv";

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<ros::Time> times;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        underwater_vehicle_msgs::VehicleData::ConstPtr i = m.instantiate<underwater_vehicle_msgs::VehicleData>();
        if (i != nullptr)
        {
            xs.push_back(i->x);
            ys.push_back(i->y);
            times.push_back(i->time);
        }
    }

    bag.close();

    std::cout << xs.size() << "\n";

    // now we need to find the height of the max plume value for each (x,y,time) tuple
    // we can get model data using ModelInterface

    ocean_models::FVCOM interface ("/home/dev/Documents/model-data/axial/");

    std::unique_ptr<ocean_models::ModelInterface> model;

    double modelTimeOffset = 0;
    double modelXOffset = 0;
    double modelYOffset = 0;

    std::string fvcom_directory = "/home/dev/Documents/model-data/axial/";
    model.reset(new ocean_models::FVCOM(fvcom_directory, &startModelLoad, &endModelLoad, 500, 500, 15, 10, 100));

    std::vector<double> zs;

    // make a vector of all valid depths
    for(int i = upper_limit; i<=lower_limit; i++)
    {
        zs.push_back(double(i));
    }

    std::cout << "Making big thing\n" << std::flush;
    std::cout << zs.size() << "\n" << std::flush;
    // for all x,y,time tuples from bag, loop over and find the plume value (dye) at each z
    std::vector<std::vector<double> > dyeMatrix(xs.size(), std::vector<double>(zs.size()));
    std::cout << "Made big thing\n" << std::flush;

    std::cout << xs.size() << "; " << ys.size() << "; " << zs.size() << "; " << times.size() << "\n" << std::flush;
    std::cout << dyeMatrix.size() << "; " << dyeMatrix[0].size() << "\n" << std::flush;

    for(int i = 0; i<xs.size(); i++)
    {
        for(int j = 0; j<zs.size(); j++)
        {
            //std::cout << interface.getDataOutOfRange(xs[i], ys[i], zs[j], times[i].toSec()) << "\n";
            //dyeMatrix[i][j] = interface.getDataOutOfRange(xs[i], ys[i], zs[j], times[i].toSec()).dye;
            dyeMatrix[i][j] = model->getDataOutOfRange(xs[i], ys[i], zs[j], times[i].toSec()).dye;
        }
    }

    // output the contents of matrix as csv
    // open file
    std::ofstream outfile;
    outfile.open(fileName);

    for(auto row: dyeMatrix)
    {
        for(auto& dye: row)
        {
            outfile << dye << ",";
        }
        outfile << "\n";
    }

    outfile.close();

    return 0;
}
