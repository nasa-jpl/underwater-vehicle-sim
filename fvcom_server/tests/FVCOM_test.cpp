#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOM.h"

#include <gtest/gtest.h>


FVCOM fvcom("test_data/box_plume_0001.nc", 10, 10, 10, 10, 10);

FVCOM fvcomMultiple("test_data/box_plume_split", 10, 10, 10, 30, 10);

TEST(FCVOMTest, GetDataMultipleFiles) {
	//Node 345
	FVCOM::FVCOMData data1 = fvcomMultiple.getData(56.69873, 25.0, 0, 0);
	FVCOM::FVCOMData data2 = fvcomMultiple.getData(56.69873, 25.0, -300, 0);

	

	//Node 17
	FVCOM::FVCOMData data3 = fvcomMultiple.getData(-50.0, -100.0, 0, 0);
	FVCOM::FVCOMData data4 = fvcomMultiple.getData(-50.0, -100.0, -28.78788, 0);
	FVCOM::FVCOMData data5 = fvcomMultiple.getData(-50.0, -100.0, -28.78788, 0.01875);

	FVCOM::FVCOMData data6 = fvcomMultiple.getData(-40.0, -100.0, 0, 0);
	FVCOM::FVCOMData data7 = fvcomMultiple.getData(-40.0, -100.0, -28.78788, 0);
	FVCOM::FVCOMData data8 = fvcomMultiple.getData(-40.0, -100.0, -28.78788, 0.01875);


	FVCOM::FVCOMData data9 = fvcomMultiple.getData(-40.0, -100.0, 0, 0.15833333);
	FVCOM::FVCOMData data10 = fvcomMultiple.getData(-40.0, -100.0, -28.78788, 0.15833333);

	FVCOM::FVCOMData data11 = fvcomMultiple.getData(-40.0, -100.0, -28.78788, 0.083333336);

	FVCOM::FVCOMData data12 = fvcomMultiple.getData(100.0, 100.0, -28.78788, 0.083333336);

	FVCOM::FVCOMData data13 = fvcomMultiple.getData(-100.0, -100.0, -28.78788, 0.083333336);


	ASSERT_FLOAT_EQ(3.2964647, data1.temp);
	ASSERT_FLOAT_EQ(34.71025, data1.salt);

	ASSERT_FLOAT_EQ(2.6035352, data2.temp);
	ASSERT_FLOAT_EQ(34.759747, data2.salt);



	ASSERT_FLOAT_EQ(3.2964647, data3.temp);
	ASSERT_FLOAT_EQ(34.71025, data3.salt);

	ASSERT_FLOAT_EQ(3.2328281, data4.temp);
	ASSERT_FLOAT_EQ(34.714798, data4.salt);

	ASSERT_FLOAT_EQ(3.2328043, data5.temp);
	ASSERT_FLOAT_EQ(34.714607, data5.salt);

	ASSERT_FLOAT_EQ(3.2964647, data6.temp);
	ASSERT_FLOAT_EQ(34.71025, data6.salt);

	ASSERT_FLOAT_EQ(3.2328281, data7.temp);
	ASSERT_FLOAT_EQ(34.714798, data7.salt);

	ASSERT_FLOAT_EQ(3.2328043, data8.temp);
	ASSERT_FLOAT_EQ(34.714607, data8.salt);

	ASSERT_FLOAT_EQ(3.2962615, data9.temp);
	ASSERT_FLOAT_EQ(34.708626, data9.salt);

	ASSERT_FLOAT_EQ(3.2337134, data10.temp);
	ASSERT_FLOAT_EQ(34.713173, data10.salt);

	ASSERT_FLOAT_EQ(3.2337518, data11.temp);
	ASSERT_FLOAT_EQ(34.713943, data11.salt);

	ASSERT_FLOAT_EQ(3.2301795, data12.temp);
	ASSERT_FLOAT_EQ(34.713943, data12.salt);

	ASSERT_FLOAT_EQ(3.230265, data13.temp);
	ASSERT_FLOAT_EQ(34.713943, data13.salt);
}

TEST(FCVOMTest, GetData) {

	//Node 345
	FVCOM::FVCOMData data1 = fvcom.getData(56.69873, 25.0, 0, 0);
	FVCOM::FVCOMData data2 = fvcom.getData(56.69873, 25.0, -300, 0);

	ASSERT_FLOAT_EQ(3.2964647, data1.temp);
	ASSERT_FLOAT_EQ(34.71025, data1.salt);

	ASSERT_FLOAT_EQ(2.6035352, data2.temp);
	ASSERT_FLOAT_EQ(34.759747, data2.salt);

	//Node 17
	FVCOM::FVCOMData data3 = fvcom.getData(-50.0, -100.0, 0, 0);
	FVCOM::FVCOMData data4 = fvcom.getData(-50.0, -100.0, -28.78788, 0);
	FVCOM::FVCOMData data5 = fvcom.getData(-50.0, -100.0, -28.78788, 0.01875);

	FVCOM::FVCOMData data6 = fvcom.getData(-40.0, -100.0, 0, 0);
	FVCOM::FVCOMData data7 = fvcom.getData(-40.0, -100.0, -28.78788, 0);
	FVCOM::FVCOMData data8 = fvcom.getData(-40.0, -100.0, -28.78788, 0.01875);

	FVCOM::FVCOMData data9 = fvcom.getData(-40.0, -100.0, 0, 0.15833333);
	FVCOM::FVCOMData data10 = fvcom.getData(-40.0, -100.0, -28.78788, 0.15833333);

	FVCOM::FVCOMData data11 = fvcomMultiple.getData(-40.0, -100.0, -28.78788, 0.083333336);

	ASSERT_FLOAT_EQ(3.2964647, data3.temp);
	ASSERT_FLOAT_EQ(34.71025, data3.salt);

	ASSERT_FLOAT_EQ(3.2328281, data4.temp);
	ASSERT_FLOAT_EQ(34.714798, data4.salt);

	ASSERT_FLOAT_EQ(3.2328043, data5.temp);
	ASSERT_FLOAT_EQ(34.714607, data5.salt);

	ASSERT_FLOAT_EQ(3.2964647, data6.temp);
	ASSERT_FLOAT_EQ(34.71025, data6.salt);

	ASSERT_FLOAT_EQ(3.2328281, data7.temp);
	ASSERT_FLOAT_EQ(34.714798, data7.salt);

	ASSERT_FLOAT_EQ(3.2328043, data8.temp);
	ASSERT_FLOAT_EQ(34.714607, data8.salt);

	ASSERT_FLOAT_EQ(3.2962615, data9.temp);
	ASSERT_FLOAT_EQ(34.708626, data9.salt);

	ASSERT_FLOAT_EQ(3.2337134, data10.temp);
	ASSERT_FLOAT_EQ(34.713173, data10.salt);

	ASSERT_FLOAT_EQ(3.2337518, data11.temp);
	ASSERT_FLOAT_EQ(34.713943, data11.salt);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}