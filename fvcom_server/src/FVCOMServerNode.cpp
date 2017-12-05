#include "ros/ros.h"
#include "fvcom_server/GetFVCOMData.h"
#include "fvcom_server/FVCOM.h"


FVCOM fvcom;

bool getFVCOMData(fvcom_server::GetFVCOMData::Request &req,
				  fvcom_server::GetFVCOMData::Response &res)
{
	FVCOM::FVCOMData data = fvcom.getData(req.x, req.y, req.h, req.time);
	
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

    fvcom = FVCOM(argv[1], 1000, 1000, 10, 10, 100);

    ros::ServiceServer service = n.advertiseService("get_fvcom_data", getFVCOMData);
  	ROS_INFO("FVCOM Model Loaded");

    ros::spin();
}: