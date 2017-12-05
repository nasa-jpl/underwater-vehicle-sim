#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOM.h"

#include <gtest/gtest.h>


FVCOM fvcomMultiple("test_data/axial_data_test", 1000, 1000, 10, 3, 10);

TEST(FVCOMTest, GetDataMultipleFiles) {

	//Test node with times in two files
	//Node: 1000
	//h: 2737.259485
	//siglay: 15
	//time: 4 in 0001_1
	FVCOM::FVCOMData data1 = fvcomMultiple.getData(8545.73568, -132697.938, 0, 0);
	FVCOM::FVCOMData data2 = fvcomMultiple.getData(8545.73568, -132697.938, -334.07498037, 0);
	FVCOM::FVCOMData data3 = fvcomMultiple.getData(8545.73568, -132697.938, -334.07498037, 0.375);

	//Test triangle with times in two files
	//Triangle: 1000
	//h: 3123.6463423333335
	//siglay: 15
	//time: 4 in 0001_1
	FVCOM::FVCOMData data4 = fvcomMultiple.getData(-138453.56466666667, -25886.79643333335, 0, 0);
	FVCOM::FVCOMData data5 = fvcomMultiple.getData(-138453.56466666667, -25886.79643333335, -381.232432006, 0);
	FVCOM::FVCOMData data6 = fvcomMultiple.getData(-138453.56466666667, -25886.79643333335, -381.232432006, 0.375);


	ASSERT_FLOAT_EQ(3.8386462639531507, data1.temp);
	ASSERT_FLOAT_EQ(34.3129397553773, data1.salt);
	ASSERT_FLOAT_EQ(0.0, data1.dye);

	ASSERT_FLOAT_EQ(3.552034178027694, data2.temp);
	ASSERT_FLOAT_EQ(34.3561324173774, data2.salt);
	ASSERT_FLOAT_EQ(0.0, data2.dye);

	ASSERT_FLOAT_EQ(3.5520324460792474, data3.temp);
	ASSERT_FLOAT_EQ(34.35613267838961, data3.salt);
	ASSERT_FLOAT_EQ(0.0, data3.dye);

	ASSERT_FLOAT_EQ(0.0, data4.u);
	ASSERT_FLOAT_EQ(0.0, data4.v);

	ASSERT_FLOAT_EQ(0.0, data5.u);
	ASSERT_FLOAT_EQ(0.0, data5.v);

	ASSERT_FLOAT_EQ(0.00038683573810392144, data6.u);
	ASSERT_FLOAT_EQ(-0.0019926347599095754, data6.v);


}

TEST(FVCOMTest, ModelEdge) {
	//Test node at edge of model
	//Node: 180
	FVCOM::FVCOMData data = fvcomMultiple.getData(150000, 150000, 0, 0);
	ASSERT_FLOAT_EQ(3.8387627679688694, data.temp);
	ASSERT_FLOAT_EQ(34.31292219813244, data.salt);
	ASSERT_FLOAT_EQ(0.0, data.dye);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}