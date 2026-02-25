#include "ros/ros.h"

#include "ocean_model_interfaces/model_interface/ModelInterface.h"
#include "ocean_model_interfaces/general_models/LinearModel.h"
#include "ocean_model_interfaces/general_models/ConstantModel.h"
#include "ocean_model_interfaces/general_models/OceanFrontModel.h"
#include "ocean_model_interfaces/fvcom/FVCOM.h"
#include "ocean_model_interfaces/geodetic_grid/GeodeticGrid.h"
#include "ocean_model_interfaces/geodetic_grid/GeodeticGridParameters.h"

#include "vehicles/Vehicle.h"
#include "std_msgs/Float64.h"

using namespace ocean_model_interfaces;

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

        float offsetX = 0;
        float offsetY = 0;
        float offsetHeight = 0;
        float offsetTime = 0;

        nhPriv.getParam("/model/offset_x", offsetX);
        nhPriv.getParam("/model/offset_y", offsetY);
        nhPriv.getParam("/model/offset_height", offsetHeight);
        nhPriv.getParam("/model/offset_time", offsetTime);

        if(model_type == "FVCOM" || model_type == "fvcom")
        {
            std::string fvcom_directory;
            if(!nhPriv.getParam("/model/fvcom_directory", fvcom_directory))
            {
                ROS_FATAL("Parameter \"/model/fvcom_directory\" not present in the parameter server.");
                exit(1);
            }

            model.reset(new FVCOM(fvcom_directory, startModelLoad, endModelLoad, 500, 500, 10, 10, 100));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("FVCOM Model Loaded: %s", fvcom_directory.c_str());
        }
        else if(model_type == "geodetic_grid") {
            std::string modelDirectory;
            double latOrigin;
            double lonOrigin;
            if(!nhPriv.getParam("/model/directory", modelDirectory))
            {
                ROS_FATAL("Parameter \"/model/directory\" not present in the parameter server.");
                exit(1);
            }

            if(!nhPriv.getParam("/model/lat_origin", latOrigin))
            {
                ROS_FATAL("Parameter \"/model/lat_origin\" not present in the parameter server.");
                exit(1);
            }

            if(!nhPriv.getParam("/model/lon_origin", lonOrigin))
            {
                ROS_FATAL("Parameter \"/model/lon_origin\" not present in the parameter server.");
                exit(1);
            }

            GeodeticGridParameters parameters;
            parameters.modelDirectory = modelDirectory;
            parameters.startLoad = startModelLoad;
            parameters.endLoad = endModelLoad;

            int timeChunkSize = parameters.timeChunkSize;
            int depthChunkSize = parameters.depthChunkSize;
            int latChunkSize = parameters.latChunkSize;
            int lonChunkSize = parameters.lonChunkSize;

            nhPriv.getParam("/model/time_chunk_size", timeChunkSize);
            nhPriv.getParam("/model/depth_chunk_size", depthChunkSize);
            nhPriv.getParam("/model/lat_chunk_size", latChunkSize);
            nhPriv.getParam("/model/lon_chunk_size", lonChunkSize);

            parameters.timeChunkSize = timeChunkSize;
            parameters.depthChunkSize = depthChunkSize;
            parameters.latChunkSize = latChunkSize;
            parameters.lonChunkSize = lonChunkSize;

            model.reset(new GeodeticGrid(parameters));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);
            model->setOrigin(Point(lonOrigin, latOrigin, 0));

            ROS_INFO("Geodetic Grid Model Loaded: %s", modelDirectory.c_str());
        }
        else if(model_type == "constant")
        {
            ConstantModel::Parameters parameters;

            nhPriv.getParam("/model/u", parameters.u);
            nhPriv.getParam("/model/v", parameters.v);
            nhPriv.getParam("/model/temp", parameters.temp);
            nhPriv.getParam("/model/salt", parameters.salt);
            nhPriv.getParam("/model/dye", parameters.dye);
            nhPriv.getParam("/model/depth", parameters.depth);

            std::cout << parameters.depth << std::endl;
            model.reset(new ConstantModel(parameters));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("Constant Model Loaded");
        } 
        else if(model_type == "linear")
        {
            LinearModel::Parameters parameters;

            nhPriv.getParam("/model/centerX", parameters.centerX);
            nhPriv.getParam("/model/centerY", parameters.centerY);
            nhPriv.getParam("/model/centerZ", parameters.centerZ);
            nhPriv.getParam("/model/zeroDistance", parameters.zeroDistance);

            std::string typeStr;
            nhPriv.getParam("/model/type", typeStr);

            if(typeStr == "euclidean") {
                parameters.type = LinearModel::DistanceFunction::EUCLIDEAN;
            } else if(typeStr == "manhattan") {
                parameters.type = LinearModel::DistanceFunction::MANHATTAN;
            }

            nhPriv.getParam("/model/u", parameters.u);
            nhPriv.getParam("/model/v", parameters.v );
            nhPriv.getParam("/model/temp", parameters.temp);
            nhPriv.getParam("/model/salt", parameters.salt);
            nhPriv.getParam("/model/dye", parameters.dye);
            nhPriv.getParam("/model/depth", parameters.depth);

            model.reset(new LinearModel(parameters));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("Linear Model Loaded");
        } 
        else if(model_type == "front") {
            OceanFrontModel::Parameters parameters;

            nhPriv.getParam("/model/front_x", parameters.frontX);
            nhPriv.getParam("/model/front_y", parameters.frontY);
            nhPriv.getParam("/model/front_orientation", parameters.frontOrientation);
            nhPriv.getParam("/model/front_width", parameters.frontWidth);

            nhPriv.getParam("/model/depths", parameters.depths);
            nhPriv.getParam("/model/side1_temp", parameters.side1Temps);
            nhPriv.getParam("/model/side2_temp", parameters.side2Temps);
            nhPriv.getParam("/model/side1_salt", parameters.side1Salts);
            nhPriv.getParam("/model/side2_salt", parameters.side2Salts);

            nhPriv.getParam("/model/current_u", parameters.currentU);
            nhPriv.getParam("/model/current_v", parameters.currentV);
            nhPriv.getParam("/model/dye", parameters.dye);

            model.reset(new OceanFrontModel(parameters));
            model->setOffsets(offsetX, offsetY, offsetHeight, offsetTime);

            ROS_INFO("Ocean Front Model Loaded");
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
    
    ros::ServiceServer service = nh.advertiseService("get_info", &getVehicleInfo);;

    //Run the simulation loop
    while(ros::ok())
    {
    	vehicle->update();

        ros::spinOnce();
        r.sleep();
    }
}