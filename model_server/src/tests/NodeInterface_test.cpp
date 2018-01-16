#include <gtest/gtest.h>

#include "ros/ros.h"
#include "model_server/GetModelData.h"

ros::ServiceClient client;

TEST(NodeInterface, GetFVCOMDataTest){
	model_server::GetModelData srv1;
	model_server::GetModelData srv2;
	model_server::GetModelData srv3;

	model_server::GetModelData srv4;
	model_server::GetModelData srv5;
	model_server::GetModelData srv6;

	model_server::GetModelData srv7;
	model_server::GetModelData srv8;

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

	srv7.request.x = 12314;
	srv7.request.y = -9648;
	srv7.request.h = -89;
	srv7.request.time = 0.11;

	srv8.request.x = -96.5869768;
	srv8.request.y = 50.2484645;
	srv8.request.h = -70;
	srv8.request.time = 0;


	bool exists = client.waitForExistence(ros::Duration(5));

	ASSERT_TRUE(exists); //Checks to make sure the service has started correctly
	
	client.call(srv1);
	client.call(srv2);
	client.call(srv3);

	client.call(srv4);
	client.call(srv5);
	client.call(srv6);

	client.call(srv7);
	client.call(srv8);

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

	ASSERT_FLOAT_EQ(3.7692957782, srv7.response.temp);
	ASSERT_FLOAT_EQ(34.3233909259, srv7.response.salt);
	ASSERT_FLOAT_EQ(0.0, srv7.response.dye);
	EXPECT_NEAR(-5.013343914645146E-4, srv7.response.u, 0.0000000001);
	EXPECT_NEAR(8.049376608763285E-4 , srv7.response.v, 0.0000000001);
    

	ASSERT_FLOAT_EQ(3.78614325, srv8.response.temp);
    ASSERT_FLOAT_EQ(34.320852, srv8.response.salt);
    ASSERT_FLOAT_EQ(0.0, srv8.response.dye);
   
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  //Initalize ROS components
  ros::init(argc, argv, "node_interface_test");

  ros::NodeHandle n;

  client = n.serviceClient<model_server::GetModelData>("get_model_data");

  return RUN_ALL_TESTS();
}
