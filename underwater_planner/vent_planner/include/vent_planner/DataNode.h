#ifndef DATA_NODE_H
#define DATA_NODE_H

#include <map>
#include <vector>
#include <cmath>
#include <sstream>

class DataTree;

#include "planner_framework/VehiclePose.h"
#include "planner_framework/PlannerData.h"

class DataNode
{
    friend class DataTree;

public:
    DataNode(DataNode* parentNode, const unsigned int nodeLevel, VehiclePose origin, double nodeSize, const unsigned nodeIndex);
    DataNode(const DataNode& other);
    ~DataNode();

    VehiclePose getCenterLocation();
    /**
     * Partition the node by the given partition factor
     * @param partitionFactor
     */
    void partition(int partitionFactor);
    bool isPartitioned();
    std::vector<DataNode*> getInitalizedNeighbors();
    const double getHeightOfPlume();
    const double getSize();
    const PlannerData& getMaxVal() const;
    const unsigned int getNodeLevel();

    /**
     * Clears all the data from the Node and all children nodes
     */
    void clear();

    struct PointerCompare
    {
        bool operator() (const DataNode* lhs, const DataNode* rhs) const;
    };
    bool operator<(const DataNode& rhs) const;
    bool operator==(const DataNode& rhs) const;

    void addData(const PlannerData& plumeData);
private:

    /**
     * Gets the data from this node
     * @param data Vector to add data to
     */
    void getData(std::vector<PlannerData*>& data);
    

    /**
     * Gets all maxima
     * Gets maxima for this node and all children. Must be surrounded by all 8 neighbors
     * @return All maxima
     */
    std::vector<DataNode*> getMaxima();

    /**
     * Gets all nodes that could be a maxima, but are not fully surrounded
     * @return All potential maxima
     */
    std::vector<DataNode*> getPotentialMaxima();

    /**
     * Creates a child and returns it. Returns existing child if one exists
     * @param nodeIndex index of the child
     * @return The created child
     */
    DataNode* createAndGetChild(unsigned int nodeIndex);

    /**
     * Gets child if it exists, otherwise returns null
     * @param nodeIndex Child to get
     * @return The child
     */
    DataNode* getChild(unsigned int nodeIndex);

    /**
     * Gets the smallest node at the given location
     * @param location
     * @return
     */
    DataNode& getSmallestNode(const VehiclePose& location);

    /**
     * Gets the closest node origin to location that is on targetNodeLevel
     * @param location
     * @param targetNodeLevel
     * @return
     */
    VehiclePose getClosestNodeOrigin(const VehiclePose& location, unsigned int targetNodeLevel);

    /**
     * Checks if this node is a maximum
     * @return
     */
    bool isMaximum();

    /**
     * Checks if this node could be a maximum, but is currently not
     * @return
     */
    bool isPotentialMaximum();

    /**
     * Creates a child at the given node index
     * @param nodeIndex
     */
    void createChild(unsigned int nodeIndex);

    /**
     * Gets the node index of the node at the given point
     * @param point
     * @return
     */
    unsigned int toNodeIndex(const VehiclePose& point);

    /**
     * Gets the node index for the given xNode and yNode values
     * @param xNode
     * @param yNode
     * @return
     */
    unsigned int toNodeIndex(const int xNode, const int yNode);

    /**
     * Gets the x,y origin of the given nodeIndex
     * @param nodeIndex
     * @return
     */
    VehiclePose toOriginXY(unsigned int nodeIndex);

    /**
     * Gets the x,y indicies of the given nodeIndex
     * @param nodeIndex
     * @param x
     * @param y
     */
    void toNodeXY(unsigned int nodeIndex, int& x, int& y);

    /**
     * Gets a child relative to the given node index child
     * @param relativeX
     * @param relativeY
     * @param nodeIndex
     * @return
     */
    DataNode* getRelativeToChild(int relativeX, int relativeY, unsigned int nodeIndex);

    /**
     * Relative helper function
     * @param node
     * @param relative
     * @return
     */
    int getParentRelative(int node, int relative);

    /**
     * Relative helper function
     * @param node
     * @param relative
     * @return
     */
    int getChildRelative(int node, int relative);

    /**
     * Gets the node relative to the current node
     * @param relativeX
     * @param relativeY
     * @return
     */
    DataNode* getRelative(int relativeX, int relativeY);

    
private:
    int partitionFactor;
    bool partitioned;
    std::map<unsigned int, DataNode> nodes;
    DataNode* parentNode;

    std::vector<PlannerData> data;
    PlannerData maxVal;

    const unsigned int nodeIndex;
    const VehiclePose origin;
    const double nodeSize;
    const unsigned int nodeLevel;

    const std::string dataMember;
};

#endif