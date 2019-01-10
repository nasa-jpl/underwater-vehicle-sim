#include <gtest/gtest.h>

#include "ros/ros.h"
#include <cmath>

#include "underwater_vehicle_msgs/VehicleData.h"
#include "underwater_vehicle_msgs/GetVehicleInfo.h"

#include "data_server/ClearData.h"
#include "data_server/GetLatestData.h"
#include "data_server/GetData.h"
#include "data_server/PlumeData.h"
#include "data_server/GetPlumeData.h"


#include "std_msgs/String.h"

ros::ServiceClient getDataClient;
ros::ServiceClient getPlumeDataClient;
ros::ServiceClient clearDataClient;
ros::ServiceClient getLatestDataClient;
ros::Publisher saveData;
ros::Publisher v1SendData;
ros::Publisher v2SendData;
ros::ServiceServer infoService;

bool getVehicleInfo(underwater_vehicle_msgs::GetVehicleInfo::Request &req,
				  	underwater_vehicle_msgs::GetVehicleInfo::Response &res)
{
	res.moduleNames.push_back("broadcaster");
	res.moduleTypes.push_back("DataBroadcaster");
	
	return true;
}

TEST(DataServerNodeInterface, PutAndGetData)
{
	ros::NodeHandle nh;

	//Clear all data out of the server
	data_server::ClearData clearSrvV1;
	data_server::ClearData clearSrvV2;

	clearSrvV1.request.name = "v1";
	clearSrvV2.request.name = "v2";

	clearDataClient.call(clearSrvV1);
	clearDataClient.call(clearSrvV2);

	std::vector<underwater_vehicle_msgs::VehicleData> allData;

	for(unsigned int i = 0; i < 8; i++)
	{
		underwater_vehicle_msgs::VehicleData newData;
		if(i < 4)
		{
			newData.name = "v1";
		}
		else
		{
			newData.name = "v2";
		}

		newData.x = i + 0.1;
		newData.y = i + 0.2;
		newData.h = i + 0.3;
		newData.time = ros::Time(i);
		newData.temp = i + 0.4;
		newData.salt = i + 0.5;
		newData.dye = i + 0.6;
		newData.sonarDepth = i + 0.7;

		allData.push_back(newData);	
	}

	for(underwater_vehicle_msgs::VehicleData data : allData)
	{
		if(data.name == "v1")
		{
			v1SendData.publish(data);
		}
		else
		{
			v2SendData.publish(data);
		}
		ros::spinOnce();
	}

	data_server::GetData v1RetrievedData1;
	v1RetrievedData1.request.name = "v1";
	v1RetrievedData1.request.start_time = ros::Time(0);
	v1RetrievedData1.request.end_time = ros::Time(8);

	data_server::GetData v1RetrievedData2;
	v1RetrievedData2.request.name = "v1";
	v1RetrievedData2.request.start_time = ros::Time(1);
	v1RetrievedData2.request.end_time = ros::Time(2.1);

	data_server::GetData v2RetrievedData1;
	v2RetrievedData1.request.name = "v2";
	v2RetrievedData1.request.start_time = ros::Time(4);
	v2RetrievedData1.request.end_time = ros::Time(5.9);

	data_server::GetData v2RetrievedData2;
	v2RetrievedData2.request.name = "v2";
	v2RetrievedData2.request.start_time = ros::Time(11);
	v2RetrievedData2.request.end_time = ros::Time(10);

	getDataClient.call(v1RetrievedData1);
	ros::spinOnce();

	getDataClient.call(v1RetrievedData2);
	ros::spinOnce();

	getDataClient.call(v2RetrievedData1);
	ros::spinOnce();

	getDataClient.call(v2RetrievedData2);
	ros::spinOnce();

	EXPECT_EQ(4, v1RetrievedData1.response.x.size());
	for(unsigned int i = 0; i < v1RetrievedData1.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[i].x, v1RetrievedData1.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[i].y, v1RetrievedData1.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[i].h, v1RetrievedData1.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[i].time.toSec(), v1RetrievedData1.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[i].temp, v1RetrievedData1.response.temp[i]);
		EXPECT_DOUBLE_EQ(allData[i].salt, v1RetrievedData1.response.salt[i]);
		EXPECT_DOUBLE_EQ(allData[i].dye, v1RetrievedData1.response.dye[i]);
		EXPECT_DOUBLE_EQ(allData[i].sonarDepth, v1RetrievedData1.response.sonarDepth[i]);
	}

	unsigned int allDataStart = 1;
	EXPECT_EQ(2, v1RetrievedData2.response.x.size());
	for(unsigned int i = 0; i < v1RetrievedData2.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].x, v1RetrievedData2.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].y, v1RetrievedData2.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].h, v1RetrievedData2.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].time.toSec(), v1RetrievedData2.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].temp, v1RetrievedData2.response.temp[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].salt, v1RetrievedData2.response.salt[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].dye, v1RetrievedData2.response.dye[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].sonarDepth, v1RetrievedData2.response.sonarDepth[i]);
	}

	allDataStart = 4;
	EXPECT_EQ(2, v2RetrievedData1.response.x.size());
	for(unsigned int i = 0; i < v2RetrievedData1.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].x, v2RetrievedData1.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].y, v2RetrievedData1.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].h, v2RetrievedData1.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].time.toSec(), v2RetrievedData1.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].temp, v2RetrievedData1.response.temp[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].salt, v2RetrievedData1.response.salt[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].dye, v2RetrievedData1.response.dye[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].sonarDepth, v2RetrievedData1.response.sonarDepth[i]);
	}

	EXPECT_EQ(v2RetrievedData2.response.x.size(), 0);
}

TEST(DataServerNodeInterface, GetLatestData)
{
	//Clear all data out of the server
	data_server::ClearData clearSrvV1;
	data_server::ClearData clearSrvV2;

	clearSrvV1.request.name = "v1";
	clearSrvV2.request.name = "v2";

	clearDataClient.call(clearSrvV1);
	clearDataClient.call(clearSrvV2);

	std::vector<underwater_vehicle_msgs::VehicleData> allData;

	for(unsigned int i = 0; i < 3; i++)
	{
		underwater_vehicle_msgs::VehicleData newData;
		
		newData.name = "v1";
		newData.x = i + 0.1;
		newData.y = i + 0.2;
		newData.h = i + 0.3;
		newData.time = ros::Time(i);
		newData.temp = i + 0.4;
		newData.salt = i + 0.5;
		newData.dye = i + 0.6;
		newData.sonarDepth = i + 0.7;

		allData.push_back(newData);	
	}

	for(underwater_vehicle_msgs::VehicleData data : allData)
	{
		v1SendData.publish(data);
		ros::spinOnce();
	}

	data_server::GetLatestData v1Latest;
	v1Latest.request.name = "v1";

	data_server::GetLatestData v2Latest;
	v2Latest.request.name = "v2";

	EXPECT_TRUE(getLatestDataClient.call(v1Latest));
	ros::spinOnce();

	EXPECT_FALSE(getLatestDataClient.call(v2Latest));
	ros::spinOnce();
	
	EXPECT_DOUBLE_EQ(allData[2].x, v1Latest.response.x);
	EXPECT_DOUBLE_EQ(allData[2].y, v1Latest.response.y);
	EXPECT_DOUBLE_EQ(allData[2].h, v1Latest.response.h);
	EXPECT_DOUBLE_EQ(allData[2].time.toSec(), v1Latest.response.time.toSec());
	EXPECT_DOUBLE_EQ(allData[2].temp, v1Latest.response.temp);
	EXPECT_DOUBLE_EQ(allData[2].salt, v1Latest.response.salt);
	EXPECT_DOUBLE_EQ(allData[2].dye, v1Latest.response.dye);
	EXPECT_DOUBLE_EQ(allData[2].sonarDepth, v1Latest.response.sonarDepth);
}

TEST(DataServerNodeInterface, GetPlumeData)
{
	ros::NodeHandle nh;

	//Clear all data out of the server
	data_server::ClearData clearSrvV1;
	data_server::ClearData clearSrvV2;

	clearSrvV1.request.name = "v1";
	clearSrvV2.request.name = "v2";

	clearDataClient.call(clearSrvV1);
	clearDataClient.call(clearSrvV2);

	std::vector<underwater_vehicle_msgs::VehicleData> allData;

	for(unsigned int i = 0; i < 8; i++)
	{
		underwater_vehicle_msgs::VehicleData newData;
		if(i < 4)
		{
			newData.name = "v1";
		}
		else
		{
			newData.name = "v2";
		}

		newData.x = i + 0.1;
		newData.y = i + 0.2;
		newData.h = i + 0.3;
		newData.time = ros::Time(i);
		newData.temp = i + 0.4;
		newData.salt = i + 0.5;
		newData.dye = i + 0.6;
		newData.sonarDepth = i + 0.7;

		allData.push_back(newData);	
	}

	for(underwater_vehicle_msgs::VehicleData data : allData)
	{
		if(data.name == "v1")
		{
			v1SendData.publish(data);
		}
		else
		{
			v2SendData.publish(data);
		}
		ros::spinOnce();
	}

	data_server::GetPlumeData v1RetrievedData1;
	v1RetrievedData1.request.name = "v1";
	v1RetrievedData1.request.start_time = ros::Time(0);
	v1RetrievedData1.request.end_time = ros::Time(8);

	data_server::GetPlumeData v1RetrievedData2;
	v1RetrievedData2.request.name = "v1";
	v1RetrievedData2.request.start_time = ros::Time(1);
	v1RetrievedData2.request.end_time = ros::Time(2.1);

	data_server::GetPlumeData v2RetrievedData1;
	v2RetrievedData1.request.name = "v2";
	v2RetrievedData1.request.start_time = ros::Time(4);
	v2RetrievedData1.request.end_time = ros::Time(5.9);

	data_server::GetPlumeData v2RetrievedData2;
	v2RetrievedData2.request.name = "v2";
	v2RetrievedData2.request.start_time = ros::Time(11);
	v2RetrievedData2.request.end_time = ros::Time(10);

	getPlumeDataClient.call(v1RetrievedData1);
	ros::spinOnce();

	getPlumeDataClient.call(v1RetrievedData2);
	ros::spinOnce();

	getPlumeDataClient.call(v2RetrievedData1);
	ros::spinOnce();

	getPlumeDataClient.call(v2RetrievedData2);
	ros::spinOnce();

	EXPECT_EQ(4, v1RetrievedData1.response.x.size());
	for(unsigned int i = 0; i < v1RetrievedData1.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[i].x, v1RetrievedData1.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[i].y, v1RetrievedData1.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[i].h, v1RetrievedData1.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[i].time.toSec(), v1RetrievedData1.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[i].dye, v1RetrievedData1.response.plume_val[i]);
	}

	unsigned int allDataStart = 1;
	EXPECT_EQ(2, v1RetrievedData2.response.x.size());
	for(unsigned int i = 0; i < v1RetrievedData2.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].x, v1RetrievedData2.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].y, v1RetrievedData2.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].h, v1RetrievedData2.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].time.toSec(), v1RetrievedData2.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].dye, v1RetrievedData2.response.plume_val[i]);
	}

	allDataStart = 4;
	EXPECT_EQ(2, v2RetrievedData1.response.x.size());
	for(unsigned int i = 0; i < v2RetrievedData1.response.x.size(); i++)
	{
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].x, v2RetrievedData1.response.x[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].y, v2RetrievedData1.response.y[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].h, v2RetrievedData1.response.h[i]);
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].time.toSec(), v2RetrievedData1.response.time[i].toSec());
		EXPECT_DOUBLE_EQ(allData[allDataStart + i].dye, v2RetrievedData1.response.plume_val[i]);
	}

	EXPECT_EQ(v2RetrievedData2.response.x.size(), 0);
}

TEST(DataServerNodeInterface, SaveData)
{

}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "node_interface_test");

  ros::NodeHandle n;

  getLatestDataClient = n.serviceClient<data_server::GetLatestData>("/data_server/get_latest");
  clearDataClient = n.serviceClient<data_server::ClearData>("/data_server/clear");
  getDataClient = n.serviceClient<data_server::GetData>("/data_server/get");
  getPlumeDataClient = n.serviceClient<data_server::GetPlumeData>("/data_server/get_plume");
  saveData = n.advertise<std_msgs::String>("/data_server/save", 1000);

  v1SendData = n.advertise<underwater_vehicle_msgs::VehicleData>("/underwater_vehicle_sim/vehicles/v1/broadcaster/data", 1000);
  v2SendData = n.advertise<underwater_vehicle_msgs::VehicleData>("/underwater_vehicle_sim/vehicles/v2/broadcaster/data", 1000);

  infoService = n.advertiseService("/underwater_vehicle_sim/vehicles/get_info", &getVehicleInfo);

  while(v1SendData.getNumSubscribers() < 1 ||
  	    v2SendData.getNumSubscribers() < 1)
  {
  	ros::WallDuration sleepDur(0.1);
  	sleepDur.sleep();
  	ros::spinOnce();
  }
  ros::spinOnce();

  return RUN_ALL_TESTS();
}