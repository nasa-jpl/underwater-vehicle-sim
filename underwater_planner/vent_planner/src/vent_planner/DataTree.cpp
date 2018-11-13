#include "vent_planner/DataTree.h"

#include "vent_planner/DataNode.h"

DataTree::DataTree(VehiclePose center, unsigned int size) :
    root(nullptr, 0, VehiclePose(center.getX() - size / 2, center.getY() - size / 2, center.getZ()), size, 0)
{}

DataTree::~DataTree() {}

DataNode& DataTree::getRoot()
{
    return root;
}

void DataTree::addData(const PlannerData& data)
{
    root.addData(data);
}

DataNode& DataTree::getSmallestNode(const VehiclePose& location)
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

const VehiclePose DataTree::getClosestNodeOrigin(const VehiclePose& location, unsigned int targetNodeLevel)
{
    return root.getClosestNodeOrigin(location, targetNodeLevel);
}