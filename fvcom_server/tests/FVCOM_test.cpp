#include "fvcom_server/fvcom.h"
#include <gtest/gtest.h>


TEST(FCVOMTest, Initalize) {
    FVCOM fvcom("test_data/box_plume_0001.nc");

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}