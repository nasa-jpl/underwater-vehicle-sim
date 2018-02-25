#ifndef DATA_TREE_H
#define DATA_TREE_H

#include "vent_planner/DataNode.h"
#include "tf/LinearMath/Vector3.h"

class DataTree
{
public:
    DataTree(tf::Vector3 origin, unsigned int size);
    ~DataTree();

public:
    void addData(const PlumeData& data);
    std::vector<DataNode*> getMaxima();
    DataNode& getRoot();
    DataNode& getSmallestNode(const tf::Vector3& location);


private:
    DataNode root;
};

#endif