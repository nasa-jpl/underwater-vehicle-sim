#include "vent_planner/DataTree.h"
#include "vent_planner/DataNode.h"
#include "tf/LinearMath/Vector3.h"


DataTree::DataTree(tf::Vector3 center, unsigned int size) :
    root(nullptr, 0, tf::Vector3(center.getX() - size / 2, center.getY() - size / 2, center.getZ()), size, 0)
{}

DataTree::~DataTree() {}

DataNode& DataTree::getRoot()
{
    return root;
}

void DataTree::addData(const PlumeData& data)
{
    root.addData(data);
}

DataNode& DataTree::getSmallestNode(const tf::Vector3& location)
{
    return root.getSmallestNode(location);
}

std::vector<DataNode*> DataTree::getMaxima()
{
    return root.getMaxima();
}

std::vector<DataNode*> DataTree::getPotentialMaxima()
{
    return root.getPotentialMaxima();
}

const tf::Vector3 DataTree::getClosestNodeOrigin(const tf::Vector3& location, unsigned int targetNodeLevel)
{
    return root.getClosestNodeOrigin(location, targetNodeLevel);
}