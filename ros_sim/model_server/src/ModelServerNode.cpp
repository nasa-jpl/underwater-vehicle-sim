#include <memory>

#include "ros/ros.h"

#include "ocean_models/model_interface/ModelInterface.h"
#include "ocean_models/model_interface/ModelData.h"

#include "model_server/GetModelData.h"


#include "ocean_models/general_models/LinearModel.h"
#include "ocean_models/general_models/ConstantModel.h"
#include "ocean_models/general_models/OceanFrontModel.h"
#include "ocean_models/fvcom/FVCOM.h"

#include "underwater_autonomy/util/ConfigurationFile.h"

#include "std_msgs/Float64.h"
#include <string>
#include <stdexcept>

using namespace ocean_models;
std::unique_ptr<ModelInterface> model;


ros::Publisher clockSpeedPub;
ros::ServiceServer dataService;
float speedUpFactor;

double modelTimeOffset = 0;
double modelXOffset = 0;
double modelYOffset = 0;

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
        ModelData data = model->getData(req.x + modelXOffset, req.y + modelYOffset, req.h, req.time + modelTimeOffset);
        res.u = data.u;
        res.v = data.v;
        res.dye = data.dye;
        res.temp = data.temp;
        res.salt = data.salt;
        res.depth = data.depth;
    }
    catch(const std::out_of_range& e)
    {
        ROS_INFO("ModelServer: Out of Range: %f %f %f %f", req.x + modelXOffset, req.y + modelYOffset, req.h, req.time + modelTimeOffset);
        ModelData data = model->getDataOutOfRange(req.x + modelXOffset, req.y + modelYOffset, req.h, req.time + modelTimeOffset);
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

    float offsetX = 0;
    float offsetY = 0;
    float offsetZ = 0;
    float offsetTime = 0;
    n.getParam("/model/offset_x", offsetX);
    n.getParam("/model/offset_y", offsetY);
    n.getParam("/model/offset_height", offsetZ);
    n.getParam("/model/offset_time", offsetTime);

    ROS_INFO("Model time offset: %f", offsetTime);
    ROS_INFO("Model X offset: %f", offsetX);
    ROS_INFO("Model Y offset: %f", offsetY);

    if(model_type == "FVCOM" || model_type == "fvcom")
    {
        std::string fvcom_directory;
        if(!n.getParam("model/fvcom_directory", fvcom_directory))
        {
            ROS_FATAL("Parameter \"fvcom_directory\" not present in the parameter server.");
            exit(1);
        }

        model.reset(new FVCOM(fvcom_directory, startModelLoad, endModelLoad, 500, 500, 15, 10, 100));
        model->setOffsets(offsetX, offsetY, offsetZ, offsetTime);

        ROS_INFO("FVCOM Model Loaded: %s", fvcom_directory.c_str());
    }
    else if(model_type == "constant")
    {
        ConstantModel::Parameters parameters;

        n.getParam("model/u", parameters.u);
        n.getParam("model/v", parameters.v);
        n.getParam("model/temp", parameters.temp);
        n.getParam("model/salt", parameters.salt);
        n.getParam("model/dye", parameters.dye);
        n.getParam("model/depth", parameters.depth);

        model.reset(new ConstantModel(parameters));
        model->setOffsets(offsetX, offsetY, offsetZ, offsetTime);

        ROS_INFO("Constant Model Loaded");
    }
    else if(model_type == "linear")
    {
        LinearModel::Parameters parameters;

        n.getParam("model/centerX", parameters.centerX);
        n.getParam("model/centerY", parameters.centerY);
        n.getParam("model/centerZ", parameters.centerZ);
        n.getParam("model/zeroDistance", parameters.zeroDistance);

        std::string typeStr;
        n.getParam("model/type", typeStr);

        if(typeStr == "euclidean") {
            parameters.type = LinearModel::DistanceFunction::EUCLIDEAN;
        } else if(typeStr == "manhattan") {
            parameters.type = LinearModel::DistanceFunction::MANHATTAN;
        }

        n.getParam("model/u", parameters.u);
        n.getParam("model/v", parameters.v );
        n.getParam("model/temp", parameters.temp);
        n.getParam("model/salt", parameters.salt);
        n.getParam("model/dye", parameters.dye);
        n.getParam("model/depth", parameters.depth);

        model.reset(new LinearModel(parameters));
        model->setOffsets(offsetX, offsetY, offsetZ, offsetTime);

        ROS_INFO("Linear Model Loaded");
    } else if(model_type == "front") {
        std::string paramFilename;
        n.getParam("model/param_file", paramFilename);

        underwater_autonomy::ConfigurationFile configFile(paramFilename);
        OceanFrontModel::Parameters parameters;

        parameters.frontX = configFile.readSimpleEntry<double>("front_x", parameters.frontX);
        parameters.frontY = configFile.readSimpleEntry<double>("front_y", parameters.frontY);
        parameters.frontOrientation = configFile.readSimpleEntry<double>("front_orientation", parameters.frontOrientation);
        parameters.frontWidth = configFile.readSimpleEntry<double>("front_width", parameters.frontWidth);

        parameters.depths = configFile.readArrayEntry<double>("depths", parameters.depths);
        parameters.side1Temps = configFile.readArrayEntry<double>("side1_temp", parameters.side1Temps);
        parameters.side2Temps = configFile.readArrayEntry<double>("side2_temp", parameters.side2Temps);
        parameters.side1Salts = configFile.readArrayEntry<double>("side1_salt", parameters.side1Salts);
        parameters.side2Salts = configFile.readArrayEntry<double>("side2_salt", parameters.side2Salts);

        parameters.currentU = configFile.readSimpleEntry<double>("current_u", parameters.currentU);
        parameters.currentV = configFile.readSimpleEntry<double>("current_v", parameters.currentV);
        parameters.dye = configFile.readSimpleEntry<double>("dye", parameters.dye);

        model.reset(new OceanFrontModel(parameters));
        model->setOffsets(offsetX, offsetY, offsetZ, offsetTime);

        ROS_INFO("Ocean Front Model Loaded");
    }  
    else
    {
        ROS_FATAL("Parameter \"model_type\" is not valid.");
        exit(1);
    }

    dataService = n.advertiseService("get_model_data", getModelData);

    ros::spin();
}
