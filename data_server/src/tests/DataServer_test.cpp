#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"

#include "ros/ros.h"

#include <experimental/filesystem>
#include <gtest/gtest.h>

namespace fs = std::experimental::filesystem;

TEST(DataServerTest, PutAndGetData)
{
    DataServer dataServer;

    std::vector<DataServerEntry> entries;
    entries.resize(4);

    entries[0].time = ros::Time(0);
    entries[1].time = ros::Time(6);
    entries[2].time = ros::Time(2);
    entries[3].time = ros::Time(11);

    entries[0].x = 0.1;
    entries[0].y = 1.2234;
    entries[0].h = 2.234;
    entries[0].temp = 3.345;
    entries[0].salt = 4.36;
    entries[0].dye = 5.564;
    entries[0].sonarDepth = 100;

    entries[1].x = 0;
    entries[1].y = 10;
    entries[1].h = 20;
    entries[1].temp = 30;
    entries[1].salt = 40;
    entries[1].dye = 50;
    entries[1].sonarDepth = 110;

    entries[2].x = -0.372;
    entries[2].y = 12.47;
    entries[2].h = -26.62345;
    entries[2].temp = 33.4567;
    entries[2].salt = -430.623451;
    entries[2].dye = 56.263412;
    entries[2].sonarDepth = 120;

    entries[3].x = 1;
    entries[3].y = -10.5;
    entries[3].h = -20.12;
    entries[3].temp = -30.526234;
    entries[3].salt = 40.2326;
    entries[3].dye = 50.23462;
    entries[3].sonarDepth = 130;

    dataServer.putData("source1", entries[0]);
    dataServer.putData("source1", entries[1]);
    dataServer.putData("source1", entries[2]);
    dataServer.putData("source1", entries[3]);

    dataServer.putData("source2", entries[0]);
    dataServer.putData("source2", entries[2]);    
    dataServer.putData("source2", entries[3]);

    std::vector<DataServerEntry>::iterator start1 = dataServer.getStartTime("source1", ros::Time(0));
    std::vector<DataServerEntry>::iterator end1 = dataServer.getEndTime("source1", ros::Time(11));

    std::vector<DataServerEntry>::iterator start2 = dataServer.getStartTime("source1", ros::Time(2));
    std::vector<DataServerEntry>::iterator end2 = dataServer.getEndTime("source1", ros::Time(9));

    std::vector<DataServerEntry>::iterator start3 = dataServer.getStartTime("source2", ros::Time(2));
    std::vector<DataServerEntry>::iterator end3 = dataServer.getEndTime("source2", ros::Time(11));

    ASSERT_EQ(3, std::distance(start1, end1));
    ASSERT_EQ(2, std::distance(start1, end2));
    ASSERT_EQ(2, std::distance(start2, end1));
    ASSERT_EQ(1, std::distance(start2, end2));

    unsigned int expectedI1[3] = {0,1,3};
    unsigned int i = 0;

    for(auto it = start1; it != end1; it++)
    {
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].x, it->x);
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].y, it->y);
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].time.toSec(), it->time.toSec());
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].h, it->h);

        ASSERT_FLOAT_EQ(entries[expectedI1[i]].temp, it->temp);
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].salt, it->salt);
        ASSERT_FLOAT_EQ(entries[expectedI1[i]].dye, it->dye);
         ASSERT_FLOAT_EQ(entries[expectedI1[i]].sonarDepth, it->sonarDepth);

        i++;
    }
    ASSERT_EQ(3, i);

    unsigned int expectedI3[3] = {2,3};
    i = 0;

    for(auto it = start3; it != end3; it++)
    {
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].x, it->x);
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].y, it->y);
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].time.toSec(), it->time.toSec());
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].h, it->h);

        ASSERT_FLOAT_EQ(entries[expectedI3[i]].temp, it->temp);
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].salt, it->salt);
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].dye, it->dye);
        ASSERT_FLOAT_EQ(entries[expectedI3[i]].sonarDepth, it->sonarDepth);

        i++;
    }
    ASSERT_EQ(2, i);
}

TEST(DataServerTest, SaveDataToCSV)
{
    DataServer dataServer;

    std::vector<DataServerEntry> entries;
    entries.resize(4);

    entries[0].time = ros::Time(0);
    entries[1].time = ros::Time(6);
    entries[2].time = ros::Time(2);
    entries[3].time = ros::Time(11);

    entries[0].x = 0.1;
    entries[0].y = 1.2234;
    entries[0].h = 2.234;
    entries[0].temp = 3.345;
    entries[0].salt = 4.36;
    entries[0].dye = 5.564;
    entries[0].sonarDepth = 100;

    entries[1].x = 0;
    entries[1].y = 10;
    entries[1].h = 20;
    entries[1].temp = 30;
    entries[1].salt = 40;
    entries[1].dye = 50;
    entries[1].sonarDepth = 110;

    entries[2].x = -0.372;
    entries[2].y = 12.47;
    entries[2].h = -26.62345;
    entries[2].temp = 33.4567;
    entries[2].salt = -430.623451;
    entries[2].dye = 56.263412;
    entries[2].sonarDepth = 120;

    entries[3].x = 1;
    entries[3].y = -10.5;
    entries[3].h = -20.12;
    entries[3].temp = -30.526234;
    entries[3].salt = 40.2326;
    entries[3].dye = 50.23462;
    entries[3].sonarDepth = 130;

    dataServer.putData("source1", entries[0]);
    dataServer.putData("source1", entries[1]);
    dataServer.putData("source1", entries[2]);
    dataServer.putData("source1", entries[3]);

    dataServer.putData("source2", entries[0]);
    dataServer.putData("source2", entries[2]);    
    dataServer.putData("source2", entries[3]);

    dataServer.saveToFile("test_data/test_file.csv");

    DataServer dataServerLoaded("test_data/test_file.csv");

    std::vector<DataServerEntry>::iterator start1Original = dataServer.getStartTime("source1", ros::Time(0));
    std::vector<DataServerEntry>::iterator end1Original = dataServer.getEndTime("source1", ros::Time(11));

    std::vector<DataServerEntry>::iterator start1Loaded = dataServerLoaded.getStartTime("source1", ros::Time(0));
    std::vector<DataServerEntry>::iterator end1Loaded = dataServerLoaded.getEndTime("source1", ros::Time(11));


    std::vector<DataServerEntry>::iterator it1 = start1Original;
    std::vector<DataServerEntry>::iterator it2 = start1Loaded;
    unsigned int i = 0;
    while(it1 != end1Original && it2 != end1Loaded)
    {
        ASSERT_FLOAT_EQ(it2->x, it1->x);
        ASSERT_FLOAT_EQ(it2->y, it1->y);
        ASSERT_FLOAT_EQ(it2->time.toSec(), it1->time.toSec());
        ASSERT_FLOAT_EQ(it2->h, it1->h);

        ASSERT_FLOAT_EQ(it2->temp, it1->temp);
        ASSERT_FLOAT_EQ(it2->salt, it1->salt);
        ASSERT_FLOAT_EQ(it2->dye, it1->dye);
        ASSERT_FLOAT_EQ(it2->sonarDepth, it1->sonarDepth);
        
        it1++;
        it2++;
        i++;
    }
    ASSERT_EQ(3, i);

    fs::remove_all("test_data");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
