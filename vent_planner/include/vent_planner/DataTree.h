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
    void addData(const PlumeDataEntry& data);

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
    DataNode& getSmallestNode(const tf::Vector3& location);

    /**
     * Gets closest node origin to the given location at the targetNodeLevel
     * @param location
     * @param targetNodeLevel
     * @return
     */
    const tf::Vector3 getClosestNodeOrigin(const tf::Vector3& location, unsigned int targetNodeLevel);


private:
    DataNode root;
};

#endif