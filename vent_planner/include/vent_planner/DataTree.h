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
    std::vector<DataNode*> getPotentialMaxima();

    DataNode& getRoot();
    DataNode& getSmallestNode(const tf::Vector3& location);
    const tf::Vector3 getClosestNodeOrigin(const tf::Vector3& location, unsigned int targetNodeLevel);


private:
    DataNode root;
};

#endif