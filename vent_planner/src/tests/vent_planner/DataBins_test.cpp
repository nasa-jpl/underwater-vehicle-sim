#include <gtest/gtest.h>

#include <math.h>

#include "tf/LinearMath/Vector3.h"

#include "plume_detector/PlumeDataEntry.h"

#include "vent_planner/DataTree.h"
#include "vent_planner/DataNode.h"

TEST(DataNode, LocalMaxima)
{
    tf::Vector3 origin(10,-10,0);
    DataTree tree(origin, 200);

    DataNode& root = tree.getRoot();
    root.partition(20);

    PlumeData data0_a(ros::Time(0), -85, -105, 0, 1);
    PlumeData data0_b(ros::Time(0), -84, -104, -18, 5);

    PlumeData data1(ros::Time(0), -74, -94, -15, 4);
    PlumeData data2(ros::Time(0), -84, -94, -12, 4);
    PlumeData data3(ros::Time(0), -74, -104, -24, 4);

    PlumeData data4(ros::Time(0), -84, -84, -15, 5);
    PlumeData data5(ros::Time(0), -84, -74, -12, 4);
    PlumeData data6(ros::Time(0), -74, -84, -24, 4);
    PlumeData data7(ros::Time(0), -74, -74, -24, 4);

    PlumeData data8(ros::Time(0), -64, -104, -15, 4);
    PlumeData data9(ros::Time(0), -64, -94, -12, 5);
    PlumeData data10(ros::Time(0), -64, -84, -24, 4);
    PlumeData data11(ros::Time(0), -54, -104, -24, 4);
    PlumeData data12(ros::Time(0), -54, -94, -24, 4);
    PlumeData data13(ros::Time(0), -54, -84, -24, 4);

    tree.addData(data0_a);
    tree.addData(data0_b);
    tree.addData(data1);
    tree.addData(data2);
    tree.addData(data3);

    tree.addData(data4);
    tree.addData(data5);
    tree.addData(data6);
    tree.addData(data7);

    tree.addData(data8);
    tree.addData(data9);
    tree.addData(data10);
    tree.addData(data11);
    tree.addData(data12);
    tree.addData(data13);

    std::vector<DataNode*> maxima = tree.getMaxima();
    std::vector<DataNode*> potentialMaxima = tree.getPotentialMaxima();

    std::vector<double> xVals {-65, -63.5, -60.5};
    std::vector<double> yVals {-95, -97.5, -90.5};

    std::vector<double> potentialMaximaX {10, -85, -85};
    std::vector<double> potentialMaximaY {-10, -105, -85};

    unsigned int partBinIndex = 0;

    for(unsigned int i = 0; i < maxima.size(); i++)
    {
        DataNode* singleMax = maxima[i];
        tf::Vector3 location = singleMax->getCenterLocation();
        if(location.getX() == xVals[0] &&
           location.getY() == yVals[0])
        {
            partBinIndex = i;
            break;
        }
    }


    maxima[partBinIndex]->partition(10);
    for(auto neighbor : maxima[partBinIndex]->getInitalizedNeighbors())
    {
        neighbor->partition(10);
    }
   
   
    //Add data in the partitioned bin
    PlumeData partData0(ros::Time(0), -63.4, -97.4, -12, 6);
    PlumeData partData1(ros::Time(0), -64.5, -98.5, -12, 5);
    PlumeData partData2(ros::Time(0), -64.5, -97.5, -12, 5);
    PlumeData partData3(ros::Time(0), -64.5, -96.5, -12, 5);
    PlumeData partData4(ros::Time(0), -63.5, -98.5, -12, 5);
    PlumeData partData5(ros::Time(0), -63.5, -96.5, -12, 5);
    PlumeData partData6(ros::Time(0), -62.5, -98.5, -12, 5);
    PlumeData partData7(ros::Time(0), -62.5, -97.5, -12, 5);
    PlumeData partData8(ros::Time(0), -62.5, -96.5, -12, 5);

    tree.addData(partData0);
    tree.addData(partData1);
    tree.addData(partData2);
    tree.addData(partData3);
    tree.addData(partData4);
    tree.addData(partData5);
    tree.addData(partData6);
    tree.addData(partData7);
    tree.addData(partData8);

    //Add data in the partitioned crossover bin
    PlumeData crossoverPartData0(ros::Time(0), -60.4, -90.4, -12, 6);
    PlumeData crossoverPartData1(ros::Time(0), -61.5, -91.5, -12, 5);
    PlumeData crossoverPartData2(ros::Time(0), -61.5, -90.5, -12, 5);
    PlumeData crossoverPartData3(ros::Time(0), -61.5, -89.5, -12, 5);
    PlumeData crossoverPartData4(ros::Time(0), -60.5, -91.5, -12, 5);
    PlumeData crossoverPartData5(ros::Time(0), -60.5, -89.5, -12, 5);
    PlumeData crossoverPartData6(ros::Time(0), -59.5, -91.5, -12, 5);
    PlumeData crossoverPartData7(ros::Time(0), -59.5, -90.5, -12, 5);
    PlumeData crossoverPartData8(ros::Time(0), -59.5, -89.5, -12, 5);

    tree.addData(crossoverPartData0);
    tree.addData(crossoverPartData1);
    tree.addData(crossoverPartData2);
    tree.addData(crossoverPartData3);
    tree.addData(crossoverPartData4);
    tree.addData(crossoverPartData5);
    tree.addData(crossoverPartData6);
    tree.addData(crossoverPartData7);
    tree.addData(crossoverPartData8);

    //Get new maxima now that more points have been added
    maxima = tree.getMaxima();

    bool inMaxima[3] = {false, false, false};
    bool inPotentialMaxima[3] = {false, false, false};

    ASSERT_EQ(3, maxima.size());
    ASSERT_EQ(3, potentialMaxima.size());

    for(auto singleMax : maxima)
    {
        tf::Vector3 location = singleMax->getCenterLocation();
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

    for(auto singleMax : potentialMaxima)
    {
        tf::Vector3 location = singleMax->getCenterLocation();
        for(unsigned int i = 0; i < potentialMaximaX.size(); i++)
        {
            if(location.getX() == potentialMaximaX[i] &&
               location.getY() == potentialMaximaY[i] &&
               !inPotentialMaxima[i])
            {
                inPotentialMaxima[i] = true;
            }
        }
    }

    ASSERT_TRUE(inMaxima[0] && inMaxima[1] && inMaxima[2]);
    ASSERT_TRUE(inPotentialMaxima[0] && inPotentialMaxima[1]);
}

TEST(DataNode, ClosestOrigin)
{
    tf::Vector3 origin(10,-10,0);
    DataTree tree(origin, 200);

    DataNode& root = tree.getRoot();
    root.partition(20);

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

    tree.addData(data0_a);
    tree.addData(data0_b);
    tree.addData(data1);
    tree.addData(data2);
    tree.addData(data3);

    tree.addData(data4);
    tree.addData(data5);
    tree.addData(data6);
    tree.addData(data7);

    tree.addData(data8);
    tree.addData(data9);
    tree.addData(data10);
    tree.addData(data11);
    tree.addData(data12);
    tree.addData(data13); 

    std::vector<tf::Vector3> actualOrigins;
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(10,-10,0), 2));
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(10,-10,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(16,-10,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(10,-16,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(16,-16,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(tf::Vector3(-20,-20,0), 0));

    std::vector<tf::Vector3> expectedOrigins;
    expectedOrigins.push_back(tf::Vector3(10,-10,0));
    expectedOrigins.push_back(tf::Vector3(10,-10,0));
    expectedOrigins.push_back(tf::Vector3(20,-10,0));
    expectedOrigins.push_back(tf::Vector3(10,-20,0));
    expectedOrigins.push_back(tf::Vector3(20,-20,0));
    expectedOrigins.push_back(tf::Vector3(-90,-110,0));

    for(unsigned long i = 0; i < actualOrigins.size(); i++)
    {
        ASSERT_EQ(expectedOrigins[i].getX(), actualOrigins[i].getX());
        ASSERT_EQ(expectedOrigins[i].getY(), actualOrigins[i].getY());
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
