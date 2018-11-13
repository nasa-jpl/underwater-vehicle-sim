#include <gtest/gtest.h>

#include <math.h>

#include "plume_detector/PlumeDataEntry.h"

#include "vent_planner/DataTree.h"
#include "vent_planner/DataNode.h"

TEST(DataNode, LocalMaxima)
{
    VehiclePose origin(10,-10,0);
    DataTree tree(origin, 200);

    DataNode& root = tree.getRoot();
    root.partition(20);

    std::map<std::string, double> oneMap = {{"plume", 1}};
    std::map<std::string, double> fourMap = {{"plume", 4}};
    std::map<std::string, double> fiveMap = {{"plume", 5}};
    std::map<std::string, double> sixMap = {{"plume", 6}};

    PlannerData data0_a(0, VehiclePose(-85, -105, 0), oneMap);
    PlannerData data0_b(0, VehiclePose(-84, -104, -18), fiveMap);

    PlannerData data1(0, VehiclePose(-74, -94, -15), fourMap);
    PlannerData data2(0, VehiclePose(-84, -94, -12), fourMap);
    PlannerData data3(0, VehiclePose(-74, -104, -24), fourMap);

    PlannerData data4(0, VehiclePose(-84, -84, -15), fiveMap);
    PlannerData data5(0, VehiclePose(-84, -74, -12), fourMap);
    PlannerData data6(0, VehiclePose(-74, -84, -24), fourMap);
    PlannerData data7(0, VehiclePose(-74, -74, -24), fourMap);

    PlannerData data8(0, VehiclePose(-64, -104, -15), fourMap);
    PlannerData data9(0, VehiclePose(-64, -94, -12), fiveMap);
    PlannerData data10(0, VehiclePose(-64, -84, -24), fourMap);
    PlannerData data11(0, VehiclePose(-54, -104, -24), fourMap);
    PlannerData data12(0, VehiclePose(-54, -94, -24), fourMap);
    PlannerData data13(0, VehiclePose(-54, -84, -24), fourMap);

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
        VehiclePose location = singleMax->getCenterLocation();
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
    PlannerData partData0(0, VehiclePose(-63.4, -97.4, -12), sixMap);
    PlannerData partData1(0, VehiclePose(-64.5, -98.5, -12), fiveMap);
    PlannerData partData2(0, VehiclePose(-64.5, -97.5, -12), fiveMap);
    PlannerData partData3(0, VehiclePose(-64.5, -96.5, -12), fiveMap);
    PlannerData partData4(0, VehiclePose(-63.5, -98.5, -12), fiveMap);
    PlannerData partData5(0, VehiclePose(-63.5, -96.5, -12), fiveMap);
    PlannerData partData6(0, VehiclePose(-62.5, -98.5, -12), fiveMap);
    PlannerData partData7(0, VehiclePose(-62.5, -97.5, -12), fiveMap);
    PlannerData partData8(0, VehiclePose(-62.5, -96.5, -12), fiveMap);

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
    PlannerData crossoverPartData0(0, VehiclePose(-60.4, -90.4, -12), sixMap);
    PlannerData crossoverPartData1(0, VehiclePose(-61.5, -91.5, -12), fiveMap);
    PlannerData crossoverPartData2(0, VehiclePose(-61.5, -90.5, -12), fiveMap);
    PlannerData crossoverPartData3(0, VehiclePose(-61.5, -89.5, -12), fiveMap);
    PlannerData crossoverPartData4(0, VehiclePose(-60.5, -91.5, -12), fiveMap);
    PlannerData crossoverPartData5(0, VehiclePose(-60.5, -89.5, -12), fiveMap);
    PlannerData crossoverPartData6(0, VehiclePose(-59.5, -91.5, -12), fiveMap);
    PlannerData crossoverPartData7(0, VehiclePose(-59.5, -90.5, -12), fiveMap);
    PlannerData crossoverPartData8(0, VehiclePose(-59.5, -89.5, -12), fiveMap);

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
        VehiclePose location = singleMax->getCenterLocation();
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
        VehiclePose location = singleMax->getCenterLocation();
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
    std::map<std::string, double> oneMap = {{"plume", 1}};
    std::map<std::string, double> fourMap = {{"plume", 4}};
    std::map<std::string, double> fiveMap = {{"plume", 5}};
    std::map<std::string, double> sixMap = {{"plume", 6}};

    VehiclePose origin(10,-10,0);
    DataTree tree(origin, 200);

    DataNode& root = tree.getRoot();
    root.partition(20);

    PlannerData data0_a(0,VehiclePose(-85, -105, 0), oneMap);
    PlannerData data0_b(0,VehiclePose(-84, -104, -18), fiveMap);

    PlannerData data1(0,VehiclePose(-74, -94, -15), fourMap);
    PlannerData data2(0,VehiclePose(-84, -94, -12), fourMap);
    PlannerData data3(0,VehiclePose(-74, -104, -24), fourMap);

    PlannerData data4(0,VehiclePose(-84, -84, -15), fiveMap);
    PlannerData data5(0,VehiclePose(-84, -74, -12), fourMap);
    PlannerData data6(0,VehiclePose(-74, -84, -24), fourMap);
    PlannerData data7(0,VehiclePose(-74, -74, -24), fourMap);

    PlannerData data8(0,VehiclePose(-64, -104, -15), fourMap);
    PlannerData data9(0,VehiclePose(-64, -94, -12), fiveMap);
    PlannerData data10(0,VehiclePose(-64, -84, -24), fourMap);
    PlannerData data11(0,VehiclePose(-54, -104, -24), fourMap);
    PlannerData data12(0,VehiclePose(-54, -94, -24), fourMap);
    PlannerData data13(0,VehiclePose(-54, -84, -24), fourMap);

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

    std::vector<VehiclePose> actualOrigins;
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(10,-10,0), 2));
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(10,-10,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(16,-10,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(10,-16,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(16,-16,0), 1));
    actualOrigins.push_back(tree.getClosestNodeOrigin(VehiclePose(-20,-20,0), 0));

    std::vector<VehiclePose> expectedOrigins;
    expectedOrigins.push_back(VehiclePose(10,-10,0));
    expectedOrigins.push_back(VehiclePose(10,-10,0));
    expectedOrigins.push_back(VehiclePose(20,-10,0));
    expectedOrigins.push_back(VehiclePose(10,-20,0));
    expectedOrigins.push_back(VehiclePose(20,-20,0));
    expectedOrigins.push_back(VehiclePose(-90,-110,0));

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
