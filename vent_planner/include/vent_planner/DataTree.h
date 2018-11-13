#ifndef DATA_TREE_H
#define DATA_TREE_H

#include "vent_planner/DataNode.h"

class DataTree
{
public:
    DataTree(VehiclePose origin, unsigned int size);
    ~DataTree();

public:
    void addData(const PlannerData& data);

    /**
     * Get all maxima nodes
     * @return
     */
    std::vector<DataNode*> getMaxima();

    /**
     * Get all nodes that could be maxima, but are not surrounded by all 8 neighbors
     * @return
     */
    std::vector<DataNode*> getPotentialMaxima();

    /**
     * Gets the root data node to the tree
     * @return
     */
    DataNode& getRoot();

    /**
     * Gets the smallest node at a given location
     * @param location
     * @return
     */
    DataNode& getSmallestNode(const VehiclePose& location);

    /**
     * Gets closest node origin to the given location at the targetNodeLevel
     * @param location
     * @param targetNodeLevel
     * @return
     */
    const VehiclePose getClosestNodeOrigin(const VehiclePose& location, unsigned int targetNodeLevel);


private:
    DataNode root;
};

#endif