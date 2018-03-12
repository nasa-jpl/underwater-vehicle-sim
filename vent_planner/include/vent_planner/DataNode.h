#ifndef DATA_NODE_H
#define DATA_NODE_H

#include <map>

class DataTree;

#include "tf/LinearMath/Vector3.h"
#include "plume_detector/PlumeData.h"

class DataNode
{
    friend class DataTree;

public:
    DataNode(DataNode* parentNode, const unsigned int nodeLevel, tf::Vector3 origin, double nodeSize, const unsigned nodeIndex);
    DataNode(const DataNode& other);
    ~DataNode();

    tf::Vector3 getCenterLocation();
    void partition(int partitionFactor);
    bool isPartitioned();
    std::vector<DataNode*> getInitalizedNeighbors();
    const double getHeightOfPlume();
    const double getSize();
    const PlumeData& getMaxVal() const;
    const unsigned int getNodeLevel();

    void clear();

    struct PointerCompare
    {
        bool operator() (const DataNode* lhs, const DataNode* rhs) const;
    };
    bool operator<(const DataNode& rhs) const;
    bool operator==(const DataNode& rhs) const;

    void addData(const PlumeData& plumeData);
private:

    void getData(std::vector<PlumeData*>& data);
    

    std::vector<DataNode*> getMaxima();
    
    DataNode* createAndGetChild(unsigned int nodeIndex);
    DataNode* getChild(unsigned int nodeIndex);

    DataNode& getSmallestNode(const tf::Vector3& location);
    tf::Vector3 getClosestNodeOrigin(const tf::Vector3& location, unsigned int targetNodeLevel);

    bool isMaximum();
    void createChild(unsigned int nodeIndex);
    DataNode* getRelativeNode(unsigned int x, unsigned int y);
    unsigned int toNodeIndex(const tf::Vector3& point);
    unsigned int toNodeIndex(const int xNode, const int yNode);
    tf::Vector3 toOriginXY(unsigned int nodeIndex);
    void toNodeXY(unsigned int nodeIndex, int& x, int& y);
    DataNode* getNode(unsigned nodeIndex);

    DataNode* getRelativeToChild(int relativeX, int relativeY, unsigned int nodeIndex);
    int getParentRelative(int node, int relative);
    int getChildRelative(int node, int relative);
    DataNode* getRelative(int relativeX, int relativeY);

    
private:
    int partitionFactor;
    bool partitioned;
    std::map<unsigned int, DataNode> nodes;
    DataNode* parentNode;

    std::vector<PlumeData> data;
    PlumeData maxVal;

    const unsigned int nodeIndex;
    const tf::Vector3 origin;
    const double nodeSize;
    const unsigned int nodeLevel;
};

#endif