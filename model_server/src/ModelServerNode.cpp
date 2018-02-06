#include <memory>

#include "ros/ros.h"

#include "model_server/ModelInterface.h"
#include "model_server/GetModelData.h"
#include "model_server/ModelData.h"

#include "constant_model/ConstantModel.h"
#include "fvcom/FVCOM.h"


std::unique_ptr<ModelInterface> model;

bool getModelData(model_server::GetModelData::Request &req,
				  model_server::GetModelData::Response &res)
{   
    ModelData data = model->getData(req.x, req.y, req.h, req.time);
    res.u = data.u;
	res.v = data.v;
	res.dye = data.dye;
	res.temp = data.temp;
	res.salt = data.salt;
    res.depth = data.depth;

	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_server");
    ros::NodeHandle n;
    
    std::string model_type;

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

        model.reset(new FVCOM(fvcom_directory, 1000, 1000, 10, 10, 100));
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
    }   
    else
    {
        ROS_FATAL("Parameter \"model_type\" is not valid.");
        exit(1);
    } 

    

    ros::ServiceServer service = n.advertiseService("get_model_data", getModelData);
  	ROS_INFO("Model Loaded");

    ros::spin();
}
