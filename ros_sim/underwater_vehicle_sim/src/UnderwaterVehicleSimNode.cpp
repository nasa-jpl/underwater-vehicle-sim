#include "ros/ros.h"

#include "ocean_models/model_interface/ModelInterface.h"
#include "ocean_models/general_models/LinearModel.h"
#include "ocean_models/general_models/ConstantModel.h"
#include "ocean_models/fvcom/FVCOM.h"


#include "vehicles/Vehicle.h"
#include "std_msgs/Float64.h"

using namespace ocean_models;

std::unique_ptr<Vehicle> vehicle;
std::unique_ptr<ModelInterface> model;

ros::Publisher clockSpeedPub;
float speedUpFactor;

void startModelLoad() 
{
    std_msgs::Float64 slowSim;
    slowSim.data = 0.1;
    clockSpeedPub.publish(slowSim);
}

void endModelLoad() 
{
    std_msgs::Float64 startSim;
    startSim.data = speedUpFactor;
    clockSpeedPub.publish(startSim);
}

bool getVehicleInfo(underwater_vehicle_msgs::GetVehicleInfo::Request &req,
				  	underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
    vehicle->getInfo(res);
	return true;
}

void loadModelLocal(ros::NodeHandle& nhPriv)
{
    bool localModel = false;
    nhPriv.getParam("local_model", localModel);
 
    if(localModel)
    {
        std::string model_type;
        if(!nhPriv.getParam("/model_type", model_type))
        {
            ROS_FATAL("Parameter \"/model_type\" not present in the parameter server.");
            exit(1);
        }

        clockSpeedPub = nhPriv.advertise<std_msgs::Float64>("/clock_server/speed_up_factor", 1, true);
        nhPriv.param<float>("/speed_up_factor", speedUpFactor, 1);

        if(model_type == "FVCOM" || model_type == "fvcom")
        {
            float offsetX = 0;
            float offsetY = 0;
            float offsetHeight = 0;
            float offsetTime = 0;

            std::string fvcom_directory;
            if(!nhPriv.getParam("/model/fvcom_directory", fvcom_directory))
            {
                ROS_FATAL("Parameter \"/model/fvcom_directory\" not present in the parameter server.");
                exit(1);
            }

            nhPriv.getParam("/model/offset_x", offsetX);
            nhPriv.getParam("/model/offset_y", offsetY);
            nhPriv.getParam("/model/offset_height", offsetHeight);
            nhPriv.getParam("/model/offset_time", offsetTime);

            model.reset(new FVCOM(fvcom_directory, &startModelLoad, &endModelLoad, 500, 500, 10, 10, 100));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("FVCOM Model Loaded: %s", fvcom_directory.c_str());
        }
        else if(model_type == "constant")
        {
            float u = 0; 
            float v = 0;
            float temp = 0;
            float salt = 0;
            float dye = 0;
            float depth = -100;

            float offsetX = 0;
            float offsetY = 0;
            float offsetHeight = 0;
            float offsetTime = 0;


            nhPriv.getParam("/model/u", u);
            nhPriv.getParam("/model/v", v);
            nhPriv.getParam("/model/temp", temp);
            nhPriv.getParam("/model/salt", salt);
            nhPriv.getParam("/model/dye", dye);
            nhPriv.getParam("/model/depth", depth);

            nhPriv.getParam("/model/offset_x", offsetX);
            nhPriv.getParam("/model/offset_y", offsetY);
            nhPriv.getParam("/model/offset_height", offsetHeight);
            nhPriv.getParam("/model/offset_time", offsetTime);

            model.reset(new ConstantModel(u, v, temp, salt, dye, depth));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);
            ROS_INFO("Constant Model Loaded");
        } 
        else if(model_type == "linear")
        {
            float u = 0; 
            float v = 0;
            float temp = 0;
            float salt = 0;
            float dye = 0;
            float depth = -100;

            float centerX = 0;
            float centerY = 0;
            float centerZ = 0;
            float zeroDistance = 100;

            float offsetX = 0;
            float offsetY = 0;
            float offsetHeight = 0;
            float offsetTime = 0;

            std::string type = "circle";

            nhPriv.getParam("/model/centerX", centerX);
            nhPriv.getParam("/model/centerY", centerY);
            nhPriv.getParam("/model/centerZ", centerZ);
            nhPriv.getParam("/model/zeroDistance", zeroDistance);
            nhPriv.getParam("/model/type", type);

            nhPriv.getParam("/model/u", u);
            nhPriv.getParam("/model/v", v);
            nhPriv.getParam("/model/temp", temp);
            nhPriv.getParam("/model/salt", salt);
            nhPriv.getParam("/model/dye", dye);
            nhPriv.getParam("/model/depth", depth);

            nhPriv.getParam("/model/offset_x", offsetX);
            nhPriv.getParam("/model/offset_y", offsetY);
            nhPriv.getParam("/model/offset_height", offsetHeight);
            nhPriv.getParam("/model/offset_time", offsetTime);

            model.reset(new LinearModel(u, v, temp, salt, dye, depth, zeroDistance, centerX, centerY, centerZ, type));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("Linear Model Loaded");
        }   
        else
        {
            ROS_FATAL("Parameter \"model_type\" is not valid.");
            exit(1);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "underwater_vehicle_sim");
    //Check and load the model locally if the option is selected
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    loadModelLocal(nhPriv);

    vehicle.reset(new Vehicle(std::move(model)));

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

    if(model != NULL)
    {
        ros::ServiceClient modelDataClient = nh.serviceClient<model_server::GetModelData>("/get_model_data");

        //Wait for model
        if(nhPriv.hasParam("/model_type"))
        {
            modelDataClient.waitForExistence();
        }
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