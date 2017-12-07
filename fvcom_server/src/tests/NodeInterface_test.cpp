#include <gtest/gtest.h>

#include "ros/ros.h"
#include "fvcom_server/GetFVCOMData.h"

ros::ServiceClient client;

TEST(NodeInterface, GetFVCOMDataTest){
	fvcom_server::GetFVCOMData srv1;
	fvcom_server::GetFVCOMData srv2;
	fvcom_server::GetFVCOMData srv3;

	fvcom_server::GetFVCOMData srv4;
	fvcom_server::GetFVCOMData srv5;
	fvcom_server::GetFVCOMData srv6;


	srv1.request.x = 8545.73568;
	srv1.request.y = -132697.938;
	srv1.request.h = 0;
	srv1.request.time = 0;

	srv2.request.x = 8545.73568;
	srv2.request.y = -132697.938;
	srv2.request.h = -334.07498037;
	srv2.request.time = 0;

	srv3.request.x = 8545.73568;
	srv3.request.y = -132697.938;
	srv3.request.h = -334.07498037;
	srv3.request.time = 0.375;

	srv4.request.x = -138453.56466666667;
	srv4.request.y = -25886.79643333335;
	srv4.request.h = 0;
	srv4.request.time = 0;

	srv5.request.x = -138453.56466666667;
	srv5.request.y = -25886.79643333335;
	srv5.request.h = -381.232432006;
	srv5.request.time = 0;

	srv6.request.x = -138453.56466666667;
	srv6.request.y = -25886.79643333335;
	srv6.request.h = -381.232432006;
	srv6.request.time = 0.375;


	bool exists = client.waitForExistence(ros::Duration(5));

	ASSERT_TRUE(exists); //Checks to make sure the service has started correctly
	
	client.call(srv1);
	client.call(srv2);
	client.call(srv3);

	client.call(srv4);
	client.call(srv5);
	client.call(srv6);


	ASSERT_FLOAT_EQ(3.8386462639531507, srv1.response.temp);
	ASSERT_FLOAT_EQ(34.3129397553773, srv1.response.salt);
	ASSERT_FLOAT_EQ(0.0, srv1.response.dye);

	ASSERT_FLOAT_EQ(3.552034178027694, srv2.response.temp);
	ASSERT_FLOAT_EQ(34.3561324173774, srv2.response.salt);
	ASSERT_FLOAT_EQ(0.0, srv2.response.dye);

	ASSERT_FLOAT_EQ(3.5520324460792474, srv3.response.temp);
	ASSERT_FLOAT_EQ(34.35613267838961, srv3.response.salt);
	ASSERT_FLOAT_EQ(0.0, srv3.response.dye);

	ASSERT_FLOAT_EQ(0.0, srv4.response.u);
	ASSERT_FLOAT_EQ(0.0, srv4.response.v);

	ASSERT_FLOAT_EQ(0.0, srv5.response.u);
	ASSERT_FLOAT_EQ(0.0, srv5.response.v);

	ASSERT_FLOAT_EQ(0.00038683573810392144, srv6.response.u);
	ASSERT_FLOAT_EQ(-0.0019926347599095754, srv6.response.v);

}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "node_interface_test");

  ros::NodeHandle n;

  client = n.serviceClient<fvcom_server::GetFVCOMData>("get_fvcom_data");

  return RUN_ALL_TESTS();
}