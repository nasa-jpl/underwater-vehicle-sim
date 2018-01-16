#include "ros/ros.h"

#include "model_server/GetModelData.h"
#include "model_server/ModelData.h"

#include "fvcom/FVCOM.h"


FVCOM fvcom;

bool getModelData(model_server::GetModelData::Request &req,
				  model_server::GetModelData::Response &res)
{    
    ModelData data = fvcom.getData(req.x, req.y, req.h, req.time);
    res.u = data.u;
	res.v = data.v;
	res.dye = data.dye;
	res.temp = data.temp;
	res.salt = data.salt;

	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fvcom_server");
    ros::NodeHandle n;

    std::string fvcom_directory;

    if(!n.getParam("fvcom_directory", fvcom_directory))
    {
    	ROS_FATAL("Parameter \"fvcom_directory\" not present in the parameter server.");
    	exit(1);
    }

    fvcom = FVCOM(fvcom_directory, 1000, 1000, 10, 10, 100);

    ros::ServiceServer service = n.advertiseService("get_model_data", getModelData);
  	ROS_INFO("FVCOM Model Loaded");

    ros::spin();
}
