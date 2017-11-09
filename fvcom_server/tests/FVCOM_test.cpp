#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOM.h"

#include <gtest/gtest.h>


FVCOM fvcomMultiple("test_data/axial_data_test", 1000, 1000, 10, 3, 10);

TEST(FCVOMTest, GetDataMultipleFiles) {

	//Test node with times in two files
	//Node: 1000
	//h: 2737.259485
	//siglay: 15
	//time: 4 in 0001_1
	FVCOM::FVCOMData data1 = fvcomMultiple.getData(8545.73568, -132697.938, 0, 0);
	FVCOM::FVCOMData data2 = fvcomMultiple.getData(8545.73568, -132697.938, -334.07498037, 0);
	FVCOM::FVCOMData data3 = fvcomMultiple.getData(8545.73568, -132697.938, -334.07498037, 0.375);

	//Test node at edge of model
	//Node: 180
	FVCOM::FVCOMData data4 = fvcomMultiple.getData(150000, 150000, 0, 0);


	ASSERT_FLOAT_EQ(3.8386462639531507, data1.temp);
	ASSERT_FLOAT_EQ(34.3129397553773, data1.salt);

	ASSERT_FLOAT_EQ(3.552034178027694, data2.temp);
	ASSERT_FLOAT_EQ(34.3561324173774, data2.salt);

	ASSERT_FLOAT_EQ(3.5520324460792474, data3.temp);
	ASSERT_FLOAT_EQ(34.35613267838961, data3.salt);

	ASSERT_FLOAT_EQ(3.8387627679688694, data4.temp);
	ASSERT_FLOAT_EQ(34.31292219813244, data4.salt);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}