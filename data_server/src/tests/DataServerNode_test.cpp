#include <gtest/gtest.h>

#include "ros/ros.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "data_server/GetData.h"
#include "std_msgs/String.h"

ros::ServiceClient client;
ros::Publisher dataRecorder;
ros::Publisher saveData;


TEST(DataServerNode, PutAndGetData)
{
  	underwater_vehicle_sim::VehicleData data1;
  	underwater_vehicle_sim::VehicleData data2;
  	underwater_vehicle_sim::VehicleData data3;

  	data1.name = "v1";
	data1.x = 0.1;
	data1.y = 1.2;
	data1.h = -65;
	data1.time = 0;
	data1.temp = 5.3;
	data1.salt = 7.6;
	data1.dye = 1.2353;

	data2.name = "v1";
	data2.x = 5.4151;
	data2.y = 4.2352;
	data2.h = -2.34;
	data2.time = 10;
	data2.temp = 54.23;
	data2.salt = 721.62;
	data2.dye = 112.2353;

	data3.name = "v1";
	data3.x = 0.112;
	data3.y = 1.4122;
	data3.h = -12.12;
	data3.time = 5;
	data3.temp = 515.3;
	data3.salt = 725.6;
	data3.dye = 125.25353;

	unsigned int subs = dataRecorder.getNumSubscribers();
	while(subs == 0)
	{
		subs = dataRecorder.getNumSubscribers();
	}
	
	dataRecorder.publish(data1);
	dataRecorder.publish(data2);
	dataRecorder.publish(data3);

	data1.name = "v2";
	data2.name = "v2";
	data3.name = "v2";

	dataRecorder.publish(data1);
	dataRecorder.publish(data3);
	dataRecorder.publish(data2);


	data_server::GetData retrievedData1;
	retrievedData1.request.name = "v1";
	retrievedData1.request.start_time = 0;
	retrievedData1.request.end_time = 10;

	data_server::GetData retrievedData2;
	retrievedData2.request.name = "v2";
	retrievedData2.request.start_time = 0;
	retrievedData2.request.end_time = 10;

	bool exists = client.waitForExistence(ros::Duration(5));
	ASSERT_TRUE(exists);

	client.call(retrievedData1);
	client.call(retrievedData2);

	ASSERT_EQ(2, retrievedData1.response.x.size());
	ASSERT_EQ(3, retrievedData2.response.x.size());

	//Data V1
	ASSERT_FLOAT_EQ(data1.x, retrievedData1.response.x[0]);
	ASSERT_FLOAT_EQ(data1.y, retrievedData1.response.y[0]);
	ASSERT_FLOAT_EQ(data1.h, retrievedData1.response.h[0]);
	ASSERT_FLOAT_EQ(data1.time, retrievedData1.response.time[0]);

	ASSERT_FLOAT_EQ(data1.temp, retrievedData1.response.temp[0]);
	ASSERT_FLOAT_EQ(data1.salt, retrievedData1.response.salt[0]);
	ASSERT_FLOAT_EQ(data1.dye, retrievedData1.response.dye[0]);


	ASSERT_FLOAT_EQ(data2.x, retrievedData1.response.x[1]);
	ASSERT_FLOAT_EQ(data2.y, retrievedData1.response.y[1]);
	ASSERT_FLOAT_EQ(data2.h, retrievedData1.response.h[1]);
	ASSERT_FLOAT_EQ(data2.time, retrievedData1.response.time[1]);

	ASSERT_FLOAT_EQ(data2.temp, retrievedData1.response.temp[1]);
	ASSERT_FLOAT_EQ(data2.salt, retrievedData1.response.salt[1]);
	ASSERT_FLOAT_EQ(data2.dye, retrievedData1.response.dye[1]);

	//Data V2
	ASSERT_FLOAT_EQ(data1.x, retrievedData2.response.x[0]);
	ASSERT_FLOAT_EQ(data1.y, retrievedData2.response.y[0]);
	ASSERT_FLOAT_EQ(data1.h, retrievedData2.response.h[0]);
	ASSERT_FLOAT_EQ(data1.time, retrievedData2.response.time[0]);

	ASSERT_FLOAT_EQ(data1.temp, retrievedData2.response.temp[0]);
	ASSERT_FLOAT_EQ(data1.salt, retrievedData2.response.salt[0]);
	ASSERT_FLOAT_EQ(data1.dye, retrievedData2.response.dye[0]);


	ASSERT_FLOAT_EQ(data3.x, retrievedData2.response.x[1]);
	ASSERT_FLOAT_EQ(data3.y, retrievedData2.response.y[1]);
	ASSERT_FLOAT_EQ(data3.h, retrievedData2.response.h[1]);
	ASSERT_FLOAT_EQ(data3.time, retrievedData2.response.time[1]);

	ASSERT_FLOAT_EQ(data3.temp, retrievedData2.response.temp[1]);
	ASSERT_FLOAT_EQ(data3.salt, retrievedData2.response.salt[1]);
	ASSERT_FLOAT_EQ(data3.dye, retrievedData2.response.dye[1]);

	ASSERT_FLOAT_EQ(data2.x, retrievedData2.response.x[2]);
	ASSERT_FLOAT_EQ(data2.y, retrievedData2.response.y[2]);
	ASSERT_FLOAT_EQ(data2.h, retrievedData2.response.h[2]);
	ASSERT_FLOAT_EQ(data2.time, retrievedData2.response.time[2]);

	ASSERT_FLOAT_EQ(data2.temp, retrievedData2.response.temp[2]);
	ASSERT_FLOAT_EQ(data2.salt, retrievedData2.response.salt[2]);
	ASSERT_FLOAT_EQ(data2.dye, retrievedData2.response.dye[2]);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "node_interface_test");

  ros::NodeHandle n;

  client = n.serviceClient<data_server::GetData>("/data_server/get");
  dataRecorder = n.advertise<underwater_vehicle_sim::VehicleData>("/data_server/put", 1000);
  saveData = n.advertise<std_msgs::String>("/data_server/save", 1000);

  return RUN_ALL_TESTS();
}