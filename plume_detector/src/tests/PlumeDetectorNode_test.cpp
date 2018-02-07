#include <gtest/gtest.h>

#include "ros/ros.h"

#include "underwater_vehicle_sim/VehicleData.h"
#include "data_server/GetData.h"
#include "plume_detector/GetPlumeData.h"
#include "std_msgs/String.h"

ros::ServiceClient client;
ros::Publisher dataRecorder;


TEST(PlumeDetectorNode, GetPlumeDetections)
{
  	plume_detector::GetPlumeData retrievedData1;
	retrievedData1.request.name = "v1";
	retrievedData1.request.start_time = ros::Time(1);
	retrievedData1.request.end_time = ros::Time(2);

	bool exists = client.waitForExistence(ros::Duration(10));
	ASSERT_TRUE(exists);

	ros::Duration(5.0).sleep();

	client.call(retrievedData1);

	ASSERT_TRUE(retrievedData1.response.time.size() > 5);

	for(unsigned int i = 0; i < retrievedData1.response.time.size(); i++)
	{
		ASSERT_TRUE(retrievedData1.response.time[i] >= ros::Time(1.0));
		ASSERT_TRUE(retrievedData1.response.time[i] <= ros::Time(2.0));

		ASSERT_FLOAT_EQ(4.0, retrievedData1.response.val[i]);
	}
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "plume_detector_node_test");

  ros::NodeHandle n;

  client = n.serviceClient<plume_detector::GetPlumeData>("/plume_detector/get");
  dataRecorder = n.advertise<underwater_vehicle_sim::VehicleData>("/data_server/put", 1000);

  return RUN_ALL_TESTS();
}