#include <memory>

#include "ros/ros.h"

#include "model_interface/ModelInterface.h"
#include "model_interface/ModelData.h"

#include "model_server/GetModelData.h"


#include "general_models/LinearModel.h"
#include "general_models/ConstantModel.h"
#include "fvcom/FVCOM.h"

#include "std_msgs/Float64.h"
#include <string>
#include <stdexcept>


std::unique_ptr<ModelInterface> model;


ros::Publisher clockSpeedPub;
ros::ServiceServer dataService;
float speedUpFactor;   

void startModelLoad()
{
    std_msgs::Float64 slowSim;
    slowSim.data = 1;
    clockSpeedPub.publish(slowSim);
}

void endModelLoad()
{
    std_msgs::Float64 startSim;
    startSim.data = speedUpFactor;
    clockSpeedPub.publish(startSim);
}


bool getModelData(model_server::GetModelData::Request &req,
				  model_server::GetModelData::Response &res)
{   
    try
    {
        ModelData data = model->getData(req.x, req.y, req.h, req.time);
        res.u = data.u;
        res.v = data.v;
        res.dye = data.dye;
        res.temp = data.temp;
        res.salt = data.salt;
        res.depth = data.depth; 
    }
    catch(const std::out_of_range& e)
    {
        ROS_INFO("ModelServer: Out of Range: %f %f %f %f", req.x, req.y, req.h, req.time);
        ModelData data = model->getDataOutOfRange(req.x, req.y, req.h, req.time);
        res.u = data.u;
        res.v = data.v;
        res.dye = data.dye;
        res.temp = data.temp;
        res.salt = data.salt;
        res.depth = data.depth;
    }
    
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_server");
    ros::NodeHandle n;
    
    std::string model_type;

    clockSpeedPub = n.advertise<std_msgs::Float64>("clock_server/speed_up_factor", 1, true);
    n.param<float>("speed_up_factor", speedUpFactor, 1);
    startModelLoad();

    if(!n.getParam("model_type", model_type))
    {
        ROS_FATAL("Parameter \"model_type\" not present in the parameter server.");
        exit(1);
    }

    if(model_type == "FVCOM" || model_type == "fvcom")
    {
        std::string fvcom_directory;
        if(!n.getParam("model/fvcom_directory", fvcom_directory))
        {
            ROS_FATAL("Parameter \"fvcom_directory\" not present in the parameter server.");
            exit(1);
        }

        model.reset(new FVCOM(fvcom_directory, &startModelLoad, &endModelLoad, 1000, 1000, 10, 10, 100));
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

        n.getParam("model/u", u);
        n.getParam("model/v", v);
        n.getParam("model/temp", temp);
        n.getParam("model/salt", salt);
        n.getParam("model/dye", dye);
        n.getParam("model/depth", depth);

        model.reset(new ConstantModel(u, v, temp, salt, dye, depth));
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
        std::string type = "circle";

        n.getParam("model/centerX", centerX);
        n.getParam("model/centerY", centerY);
        n.getParam("model/centerZ", centerZ);
        n.getParam("model/zeroDistance", zeroDistance);
        n.getParam("model/type", type);

        n.getParam("model/u", u);
        n.getParam("model/v", v);
        n.getParam("model/temp", temp);
        n.getParam("model/salt", salt);
        n.getParam("model/dye", dye);
        n.getParam("model/depth", depth);

        model.reset(new LinearModel(u, v, temp, salt, dye, depth, zeroDistance, centerX, centerY, centerZ, type));
        ROS_INFO("Linear Model Loaded");
    }   
    else
    {
        ROS_FATAL("Parameter \"model_type\" is not valid.");
        exit(1);
    }

    dataService = n.advertiseService("get_model_data", getModelData);

    ros::spin();
}
