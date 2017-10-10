#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOM.h"

#include <gtest/gtest.h>


FVCOM fvcom("test_data/box_plume_0001.nc", 10, 10, 10, 10, 10);

TEST(FCVOMTest, GetData) {

	//Node 345
	FVCOM::FVCOMData data1 = fvcom.getData(56.69873, 25.0, 0, 0);
	FVCOM::FVCOMData data2 = fvcom.getData(56.69873, 25.0, -300, 0);

	ASSERT_FLOAT_EQ(3.2964647, data1.temp);
	ASSERT_FLOAT_EQ(34.71025, data1.salt);

	ASSERT_FLOAT_EQ(2.6035352, data2.temp);
	ASSERT_FLOAT_EQ(34.759747, data2.salt);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}