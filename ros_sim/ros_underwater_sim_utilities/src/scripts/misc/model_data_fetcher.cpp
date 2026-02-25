#include "ocean_model_interfaces/model_interface/ModelData.h"

#include "ocean_model_interfaces/model_interface/ModelInterface.h"
#include "ocean_model_interfaces/fvcom/FVCOM.h"

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


int main(int argc,      // Number of strings in array argv
         char *argv[])   // Array of command-line argument strings)
{
    // need to read in bag given by user
    rosbag::Bag bag;
    if(argc < 2)
    {
        std::cout << "Not enough args: provide bag path\n";
        return 1;
    }
    bag.open(argv[1]);  // BagMode is Read by default

    int lower_limit = 2000; //2000

    int upper_limit = 100;
    std::string fileName = "out.csv";

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<ros::Time> times;

    std::cout << "Getting data from model\n";

    // take a sample, 1 out of every 10
    int progress = 0;
    std::cout << "Reading from bag\n";
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        underwater_vehicle_msgs::VehicleData::ConstPtr i = m.instantiate<underwater_vehicle_msgs::VehicleData>();
        if (i != nullptr && (progress += 1) % 10 == 0)
        {
            progress = 0;
            xs.push_back(i->x);
            ys.push_back(i->y);
            times.push_back(i->time);
        }
    }

    bag.close();

    std::cout << xs.size() << "\n";

    // now we need to find the height of the max plume value for each (x,y,time) tuple
    // we can get model data using ModelInterface

    ocean_model_interfaces::FVCOM interface ("/home/dev/Documents/model-data/axial/");

    std::unique_ptr<ocean_model_interfaces::ModelInterface> model;

    double modelTimeOffset = 0;
    double modelXOffset = 0;
    double modelYOffset = 0;

    std::string fvcom_directory = "/home/dev/Documents/model-data/axial/";
    model.reset(new ocean_model_interfaces::FVCOM(fvcom_directory, &startModelLoad, &endModelLoad, 500, 500, 15, 10, 100));

    std::vector<double> zs;

    // make a vector of all valid depths
    for(int i = upper_limit; i<=lower_limit; i += 10)
    {
        zs.push_back(double(i));
    }

    // for all x,y,time tuples from bag, loop over and find the plume value (dye) at each z
    std::vector<std::vector<double> > dyeMatrix(xs.size(), std::vector<double>(zs.size()));

	std::cout << "About to read from model\n";
    std::cout << xs.size() << "; " << ys.size() << "; " << zs.size() << "; " << times.size() << "\n";
    std::cout << dyeMatrix.size() << "; " << dyeMatrix[0].size() << "\n";

    std::stringstream outputStringStream;

	std::cout << "Writing file\n";

    std::ofstream outfile(fileName);
    outfile << "x,y,z,time,dye\n";

    for(int i = 0; i<xs.size(); i++)
    {

		if(i % 25000 == 0 && i > 0)
		{
			outfile << outputStringStream.rdbuf();
			std::stringstream().swap(outputStringStream);
		}
        for(int j = 0; j<zs.size(); j++)
        {
            try{
                dyeMatrix[i][j] = model->getData(ys[i] + modelXOffset, xs[i] + modelYOffset,
                                                -1*zs[j], times[i].toSec() + modelTimeOffset).dye;
            }
            catch(const std::out_of_range& e)
            {
                dyeMatrix[i][j] = model->getDataOutOfRange(ys[i] + modelXOffset, xs[i] + modelYOffset,
                                                -1*zs[j], times[i].toSec() + modelTimeOffset).dye;
            }

            outputStringStream << xs[i] << "," << ys[i] << "," << zs[j] << "," << times[i] << "," << dyeMatrix[i][j] << "\n";
        }
    }


    outfile << outputStringStream.rdbuf();

	outfile.close();

    return 0;
}
