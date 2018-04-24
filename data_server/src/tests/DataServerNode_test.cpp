#include <gtest/gtest.h>

#include "ros/ros.h"

#include <cmath>
#include "underwater_vehicle_sim/VehicleData.h"
#include "data_server/GetData.h"
#include "data_server/PlumeData.h"
#include "std_msgs/String.h"

ros::ServiceClient client;
ros::Publisher v1Broadcaster;
ros::Publisher v2Broadcaster;
ros::Publisher saveData;

struct PlumeHelper
{
	int correctCount;
	int incorrectCount;

	PlumeHelper() :
			correctCount(0),
			incorrectCount(0)
	{}

	void plumeCB(const data_server::PlumeData& msg)
	{

		if(fabs(100 - msg.x) > 0.0000001 ||
		   fabs(100 - msg.y) > 0.0000001 ||
		   fabs(4 - msg.plume_strength) > 0.0000001)
		{
			incorrectCount++;
		}
		else
		{
			correctCount++;
		}
	}
};



TEST(DataServerNode, PutAndGetData)
{
	ros::NodeHandle nh;
	PlumeHelper helper;
	ros::Subscriber sub = nh.subscribe("/data_server/v1/plume_data",0, &PlumeHelper::plumeCB, &helper);

	data_server::GetData retrievedData1;
	retrievedData1.request.name = "v1";
	retrievedData1.request.start_time = ros::Time(1);
	retrievedData1.request.end_time = ros::Time(2);

	bool exists = client.waitForExistence(ros::Duration(20));
	ASSERT_TRUE(exists);

	ros::Duration(5.0).sleep();

	client.call(retrievedData1);

    ros::spinOnce();

	ASSERT_TRUE(retrievedData1.response.time.size() > 5);

	for(unsigned int i = 0; i < retrievedData1.response.time.size(); i++)
	{
		ASSERT_TRUE(retrievedData1.response.time[i] >= ros::Time(1.0));
		ASSERT_TRUE(retrievedData1.response.time[i] <= ros::Time(2.0));

		ASSERT_FLOAT_EQ(100, retrievedData1.response.x[i]);
		ASSERT_FLOAT_EQ(100, retrievedData1.response.y[i]);
		ASSERT_FLOAT_EQ(3.0, retrievedData1.response.salt[i]);
		ASSERT_FLOAT_EQ(2.0, retrievedData1.response.temp[i]);
		ASSERT_FLOAT_EQ(4.0, retrievedData1.response.dye[i]);
        ASSERT_FLOAT_EQ(200.0, retrievedData1.response.sonarDepth[i]);
	}

    ros::spinOnce();

	ASSERT_TRUE(helper.correctCount >= 1);
	ASSERT_TRUE(helper.incorrectCount == 0);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "node_interface_test");

  ros::NodeHandle n;

  client = n.serviceClient<data_server::GetData>("/data_server/get");
  saveData = n.advertise<std_msgs::String>("/data_server/save", 1000);

  return RUN_ALL_TESTS();
}