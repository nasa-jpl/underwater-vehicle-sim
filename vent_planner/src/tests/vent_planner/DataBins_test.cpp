#include <gtest/gtest.h>

#include <math.h>

#include "tf/LinearMath/Vector3.h"

#include "plume_detector/PlumeData.h"
#include "vent_planner/DataBin.h"
#include "vent_planner/DataBins.h"

TEST(DataBins, LocalMaxima)
{
    tf::Vector3 origin(10,-10,0);
    DataBins dataBins(origin, 100, 10);

    PlumeData data0_a(ros::Time(0),-85, -105, 0, 1);
    PlumeData data0_b(ros::Time(0),-84, -104, -18, 5);

    PlumeData data1(ros::Time(0),-74, -94, -15, 4);
    PlumeData data2(ros::Time(0),-84, -94, -12, 4);
    PlumeData data3(ros::Time(0),-74, -104, -24, 4);

    PlumeData data4(ros::Time(0),-84, -84, -15, 5);
    PlumeData data5(ros::Time(0),-84, -74, -12, 4);
    PlumeData data6(ros::Time(0),-74, -84, -24, 4);
    PlumeData data7(ros::Time(0),-74, -74, -24, 4);

    PlumeData data8(ros::Time(0),-64, -104, -15, 4);
    PlumeData data9(ros::Time(0),-64, -94, -12, 5);
    PlumeData data10(ros::Time(0),-64, -84, -24, 4);
    PlumeData data11(ros::Time(0),-54, -104, -24, 4);
    PlumeData data12(ros::Time(0),-54, -94, -24, 4);
    PlumeData data13(ros::Time(0),-54, -84, -24, 4);

    dataBins.addData(data0_a);
    dataBins.addData(data0_b);
    dataBins.addData(data1);
    dataBins.addData(data2);
    dataBins.addData(data3);

    dataBins.addData(data4);
    dataBins.addData(data5);
    dataBins.addData(data6);
    dataBins.addData(data7);

    dataBins.addData(data8);
    dataBins.addData(data9);
    dataBins.addData(data10);
    dataBins.addData(data11);
    dataBins.addData(data12);
    dataBins.addData(data13); 

    std::vector<std::reference_wrapper<DataBin>> maxima = dataBins.getLocalMaxima();
    std::vector<std::reference_wrapper<DataBin>> averageMaxima = dataBins.getLocalAverageMaxima();


    std::vector<double> xVals {-85, -65, -83.5, -85};
    std::vector<double> yVals {-85, -95, -87.5, -105};

    unsigned int partBinIndex = 0;

    for(unsigned int i = 0; i < maxima.size(); i++)
    {
        std::reference_wrapper<DataBin> singleMax = maxima[i];
        tf::Vector3 location = singleMax.get().getCenterLocation();
        if(location.getX() == xVals[0] &&
           location.getY() == yVals[0])
        {
            partBinIndex = i;
            break;
        }
    }

    dataBins.partition(maxima[partBinIndex], 1);
    tf::Vector3 midPartition(-86, -84, 0);
    
    //Add data in the partitioned bin
    PlumeData partData0(ros::Time(0), -83.4, -87.4, -12, 6);
    PlumeData partData1(ros::Time(0), -84.5, -88.5, -12, 5);
    PlumeData partData2(ros::Time(0), -84.5, -87.5, -12, 5);
    PlumeData partData3(ros::Time(0), -84.5, -86.5, -12, 5);
    PlumeData partData4(ros::Time(0), -83.5, -88.5, -12, 5);
    PlumeData partData5(ros::Time(0), -83.5, -86.5, -12, 5);
    PlumeData partData6(ros::Time(0), -82.5, -88.5, -12, 5);
    PlumeData partData7(ros::Time(0), -82.5, -87.5, -12, 5);
    PlumeData partData8(ros::Time(0), -82.5, -86.5, -12, 5);

    dataBins.addData(partData0);
    dataBins.addData(partData1);
    dataBins.addData(partData2);
    dataBins.addData(partData3);
    dataBins.addData(partData4);
    dataBins.addData(partData5);
    dataBins.addData(partData6);
    dataBins.addData(partData7);
    dataBins.addData(partData8);

    //Get new maxima now that more points have been added
    maxima = dataBins.getLocalMaxima();
    averageMaxima = dataBins.getLocalAverageMaxima();

    bool inMaxima[4] = {false, false, false, false};
    bool inAverageMaxima[3] = {false, false, false};

    ASSERT_EQ(4, maxima.size());
    ASSERT_EQ(3, averageMaxima.size());

    for(auto singleMax : maxima)
    {
        tf::Vector3 location = singleMax.get().getCenterLocation();
        for(unsigned int i = 0; i < xVals.size(); i++)
        {
            if(location.getX() == xVals[i] &&
               location.getY() == yVals[i] &&
               !inMaxima[i])
            {
                inMaxima[i] = true;
            }
        }
    }

    for(auto singleMax : averageMaxima)
    {
        tf::Vector3 location = singleMax.get().getCenterLocation();
        for(unsigned int i = 0; i < xVals.size() - 1; i++)
        {
            if(location.getX() == xVals[i] &&
               location.getY() == yVals[i] &&
               !inAverageMaxima[i])
            {
                inAverageMaxima[i] = true;
            }
        }
    }

    ASSERT_TRUE(inMaxima[0] && inMaxima[1] && inMaxima[2] && inMaxima[3]);
    ASSERT_TRUE(inAverageMaxima[0] && inAverageMaxima[1] && inAverageMaxima[2]);
    ASSERT_EQ(1, dataBins.getSmallestBinSize(midPartition));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
