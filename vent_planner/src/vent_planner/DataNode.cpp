#include "vent_planner/DataNode.h"
#include "tf/LinearMath/Vector3.h"
#include "plume_detector/PlumeDataEntry.h"

#include "ros/ros.h"
#include <map>
DataNode::DataNode(DataNode* parentNode, const unsigned int nodeLevel, const tf::Vector3 origin, double nodeSize, const unsigned nodeIndex) :
    parentNode(parentNode),
    nodeLevel(nodeLevel),
    origin(origin),
    nodeSize(nodeSize),
    nodeIndex(nodeIndex),
    partitioned(false)
{}

DataNode::DataNode(const DataNode& other) :
    parentNode(other.parentNode),
    nodeLevel(other.nodeLevel),
    origin(other.origin),
    nodeSize(other.nodeSize),
    nodeIndex(other.nodeIndex),
    partitioned(other.partitioned),
    data(other.data),
    maxVal(other.maxVal),
    nodes(other.nodes),
    partitionFactor(other.partitionFactor)
{}

DataNode::~DataNode() {}

void DataNode::addData(const PlumeDataEntry& plumeData)
{

    if(maxVal.val <= plumeData.val)
    {
        maxVal = plumeData;
    }

    if(partitioned)
    {
        tf::Vector3 dataVector(plumeData.x, plumeData.y, plumeData.h);
        unsigned int nodeIndex = toNodeIndex(dataVector);
        createAndGetChild(nodeIndex)->addData(plumeData);
    }   
    else
    {
        data.push_back(plumeData);
    }

}

void DataNode::partition(int partitionFactor)
{
    if(!partitioned)
    {
        this->partitionFactor = partitionFactor;
        partitioned = true;
        for(auto dataPoint : data)
        {
            addData(dataPoint);
        }
        data.clear(); 
    }
    
}

tf::Vector3 DataNode::getCenterLocation()
{
    return tf::Vector3(origin.getX() + nodeSize / 2.0, origin.getY() + nodeSize / 2.0, origin.getZ());
}

DataNode& DataNode::getSmallestNode(const tf::Vector3& location)
{
    if(!partitioned)
    {
        return *this;
    }

    DataNode* child = getChild(toNodeIndex(location));
    if(!child)
    {
        return *this;
    }
    else
    {
        return child->getSmallestNode(location);
    }
}

tf::Vector3 DataNode::getClosestNodeOrigin(const tf::Vector3& location, unsigned int targetNodeLevel)
{
    tf::Vector3 baseOrigin;
    float baseSize = 0;
    if(!partitioned || targetNodeLevel == nodeLevel)
    {
        baseOrigin = origin;
        baseSize = nodeSize;
    }
    else if(partitioned)
    {
        unsigned int baseNodeIndex = toNodeIndex(location);
        DataNode* child = getChild(toNodeIndex(location));
        if(!child)
        {
            baseOrigin = toOriginXY(baseNodeIndex);
            baseSize = nodeSize / partitionFactor;
        }
        else
        {
            return child->getClosestNodeOrigin(location, targetNodeLevel);
        } 
    }
    
    float x = 0;
    float y = 0;
    if(fabs(baseOrigin.getX() - location.getX()) < fabs((baseOrigin.getX() + baseSize) - location.getX()))
    {
        x = baseOrigin.getX();
    }
    else
    {
        x = baseOrigin.getX() + baseSize;
    }

    if(fabs(baseOrigin.getY() - location.getY()) < fabs((baseOrigin.getY() + baseSize) - location.getY()))
    {
        y = baseOrigin.getY();
    }
    else
    {
        y = baseOrigin.getY() + baseSize;
    }

    return tf::Vector3(x, y, location.getZ());
}


DataNode* DataNode::getChild(unsigned int nodeIndex)
{
    if(!partitioned)
    {
        return nullptr;
    }

    auto child = nodes.find(nodeIndex);
    if(child == nodes.end())
    {
        return nullptr;
    }

    return &child->second;
}

DataNode* DataNode::createAndGetChild(unsigned int nodeIndex)
{
    if(!partitioned)
    {
        return nullptr;
    }
    
    auto child = nodes.find(nodeIndex);
    if(child == nodes.end())
    {
        createChild(nodeIndex);
        child = nodes.find(nodeIndex);
    }

    return &(child->second);
}

DataNode* DataNode::getRelative(int relativeX, int relativeY)
{
    if(!parentNode)
    {
        return nullptr;
    }

    return parentNode->getRelativeToChild(relativeX, relativeY, nodeIndex);
}

DataNode* DataNode::getRelativeToChild(int relativeX, int relativeY, unsigned int nodeIndex)
{
    if(!partitioned)
    {
        return nullptr;
    }

    int nodeX, nodeY;
    toNodeXY(nodeIndex, nodeX, nodeY);

    if(nodeX + relativeX < 0 ||
       nodeY + relativeY < 0 ||
       nodeX + relativeX >= partitionFactor ||
       nodeY + relativeY >= partitionFactor)
    {
        if(!parentNode)
        {
            return nullptr;
        }

        DataNode* neighborParent = parentNode->getRelativeToChild(getParentRelative(nodeX, relativeX), 
                                                      getParentRelative(nodeY, relativeY), 
                                                      this->nodeIndex);

        if(!neighborParent || !neighborParent->isPartitioned())
        {
            return nullptr;
        }

        unsigned int childNodeIndex = neighborParent->toNodeIndex(getChildRelative(nodeX, relativeX), 
                                                                 getChildRelative(nodeY, relativeY));

        return neighborParent->getChild(childNodeIndex);
    }

    unsigned int newNodeIndex = toNodeIndex(nodeX + relativeX, nodeY + relativeY);
    auto newNode = nodes.find(newNodeIndex);

    if(newNode == nodes.end())
    {
        return nullptr;
    }
    return &newNode->second;
}

int DataNode::getParentRelative(int node, int relative)
{
    int sign = (relative > 0) - (relative < 0);
    if(sign > 0)
    {
        return (relative + node) / partitionFactor;
    }
    else if(sign < 0)
    {
        return (relative - ((partitionFactor - 1) - node))  / partitionFactor;
    }

    return 0;
}

int DataNode::getChildRelative(int node, int relative)
{
    int sign = (relative > 0) - (relative < 0);
    if(sign > 0)
    {
        return (relative + node) % partitionFactor;
    }
    else if(sign < 0)
    {
        return (partitionFactor - 1) + (relative - ((partitionFactor - 1) - node))  % partitionFactor;
    }

    return node;
}

void DataNode::createChild(unsigned int nodeIndex)
{
    tf::Vector3 childOrigin = toOriginXY(nodeIndex);
    if(nodes.count(nodeIndex) == 0)
    {
        DataNode node(this, nodeLevel + 1, childOrigin, nodeSize / partitionFactor, nodeIndex);
        nodes.insert(std::make_pair(nodeIndex, std::move(node)));
    }
}

unsigned int DataNode::toNodeIndex(const tf::Vector3& point)
{
    if(!partitioned ||
       point.getX() < origin.getX() ||
       point.getY() < origin.getY() ||
       point.getX() - origin.getX() >= nodeSize ||
       point.getY() - origin.getY() >= nodeSize)
    {
        std::ostringstream stringStream;
        stringStream << "DataNode.toNodeIndex(): Node out of range pointX: " << point.getX() << ", pointY: " << point.getY() << ",originX: " << origin.getX() << ", originY: " << origin.getY() << ", nodeSize: " << nodeSize << ", partitionFactor: " << partitionFactor;
        std::string whatStr = stringStream.str();
        throw std::out_of_range(whatStr);
    }

    unsigned int xNode = (point.getX() - origin.getX()) / (nodeSize / partitionFactor);
    unsigned int yNode = (point.getY() - origin.getY()) / (nodeSize / partitionFactor);

    return xNode * partitionFactor + yNode;
}

unsigned int DataNode::toNodeIndex(const int xNode, const int yNode)
{
    if(!partitioned ||
       xNode >= partitionFactor ||
       yNode >= partitionFactor)
    {
        std::ostringstream stringStream;
        stringStream <<  "DataNode.toNodeIndex(): Node out of range nodeX: " << xNode << ", nodeY: " << yNode << ", partitionFactor:" << partitionFactor;
        std::string whatStr = stringStream.str();
        throw std::out_of_range(whatStr);
    }

    return xNode * partitionFactor + yNode;
}

tf::Vector3 DataNode::toOriginXY(unsigned int nodeIndex)
{
    unsigned int xNode = nodeIndex / partitionFactor;
    unsigned int yNode = nodeIndex - (xNode * partitionFactor);
 
    if(!partitioned ||
       xNode >= partitionFactor ||
       yNode >= partitionFactor)
    {
        std::ostringstream stringStream;
        stringStream <<  "DataNode.toOriginXY(): Node out of range nodeIndex: " << nodeIndex << ", partitionFactor:" << partitionFactor;
        std::string whatStr = stringStream.str();
        throw std::out_of_range(whatStr);
    }

    tf::Vector3 vec(origin.getX() + xNode * (nodeSize / partitionFactor), origin.getY() + yNode * (nodeSize / partitionFactor), origin.getZ());
    return vec;
}

void DataNode::toNodeXY(unsigned int nodeIndex, int& x, int& y)
{
    unsigned int xNode = nodeIndex / partitionFactor;
    unsigned int yNode = nodeIndex - (xNode * partitionFactor);
 
    if(!partitioned ||
       xNode >= partitionFactor ||
       yNode >= partitionFactor)
    {
        std::ostringstream stringStream;
        stringStream <<  "DataNode.toNodeXY(): Node out of range x: " << x << ", y: " << y << ", nodeIndex: " << nodeIndex << ", partitionFactor:" << partitionFactor;
        std::string whatStr = stringStream.str();
        throw std::out_of_range(whatStr);
    }

    x = xNode;
    y = yNode;
}

std::vector<DataNode*> DataNode::getMaxima()
{
    std::vector<DataNode*> maxima;
    if(isMaximum())
    {
        maxima.push_back(this);
    }

    if(partitioned)
    {
        for(auto& node : nodes)
        {
            std::vector<DataNode*> nodeMaxima = node.second.getMaxima();
            for(auto maximum : nodeMaxima)
            {
                maxima.push_back(maximum);
            }
        }
    }
    
    return maxima; 
}

std::vector<DataNode*> DataNode::getPotentialMaxima()
{
    std::vector<DataNode*> maxima;
    if(isPotentialMaximum())
    {
        maxima.push_back(this);
    }

    if(partitioned)
    {
        for(auto& node : nodes)
        {
            std::vector<DataNode*> nodeMaxima = node.second.getPotentialMaxima();
            for(auto maximum : nodeMaxima)
            {
                maxima.push_back(maximum);
            }
        }

    }

    return maxima;
}


bool DataNode::isMaximum()
{
    for(int x = -1; x <= 1; x++)
    {
        for(int y = -1; y <= 1; y++)
        {
            if(!(x == 0 && y == 0))
            {
               DataNode* neighbor = getRelative(x,y);
                if(!neighbor || neighbor->maxVal.val > maxVal.val)
                {
                    return false;
                } 
            }
        }
    }
    
    return true;
}

bool DataNode::isPotentialMaximum()
{
    int count = 0;
    for(int x = -1; x <= 1; x++)
    {
        for(int y = -1; y <= 1; y++)
        {
            if(!(x == 0 && y == 0))
            {
                DataNode* neighbor = getRelative(x,y);
                if(neighbor)
                {
                    if(neighbor->maxVal.val > maxVal.val)
                    {
                        return false;
                    }
                    else if(neighbor->maxVal.val < maxVal.val)
                    {
                        count++;
                    }
                }
            }
        }
    }

    if(count == 8)
    {
        return false;
    }
    return true;
}

std::vector<DataNode*> DataNode::getInitalizedNeighbors()
{
    std::vector<DataNode*> neighbors;
    for(int x = -1; x <= 1; x++)
    {
        for(int y = -1; y <= 1; y++)
        {
            if(!(x == 0 && y == 0))
            {
               DataNode* neighbor = getRelative(x,y);
                if(neighbor)
                {
                    neighbors.push_back(neighbor);
                } 
            }
        }
    }

    return neighbors;
}

void DataNode::getData(std::vector<PlumeDataEntry*>& allData)
{
    for(auto& d : data)
    {
        allData.push_back(&d);
    }

    if(partitioned)
    {
        for(auto& node : nodes)
        {
            node.second.getData(allData);
        }
    }
}

const double DataNode::getHeightOfPlume()
{
    std::vector<PlumeDataEntry*> allData;
    getData(allData);
    //bin data by depth return bin with largest average
    unsigned int binSize = 10;
    double minHeight = std::numeric_limits<double>::max();
    double maxHeight = -std::numeric_limits<double>::max();

    //if there is no data then there is no plume height to find
    if(allData.size() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    //calculate min and max heights for bins
    for(unsigned int i = 0; i < allData.size(); i++)
    {
        auto d = allData[i];
        if(minHeight > d->h)
        {
            minHeight = d->h;
        }

        if(maxHeight < d->h)
        {
            maxHeight = d->h;
        }
    }

    //if heights are the same then that is the only height to return
    if(minHeight == maxHeight)
    {
        return minHeight;
    }


    int numBins = ceil((maxHeight - minHeight) / binSize);

    if(numBins == 0)
    {
        return false;
    }

    ROS_INFO("Planner: GET PLUME HEIGHT, numBins: %i, maxHeight: %f, minHeight: %f, binSize: %u", numBins, maxHeight, minHeight, binSize);
    std::vector<double> bins(numBins, 0);
    std::vector<int> binCount(numBins, 0);

    for(unsigned int i = 0; i < allData.size(); i++)
    {
        auto d = allData[i];
        int bin = (d->h - minHeight) / binSize;
        bins[bin] += d->val;
        binCount[bin]++;
    }

    int maxBin = 0;
    double maxBinVal = -std::numeric_limits<double>::max();
    for(unsigned int i = 0; i < binSize; i++)
    {
        if(maxBinVal < bins[i] / binCount[i])
        {
            maxBinVal = bins[i] / binCount[i];
            maxBin = i;
        }
    }

    //calculate max bins depth
    return minHeight + (maxBin * binSize) + binSize / 2;
}

const double DataNode::getSize()
{
    return nodeSize;
}

void DataNode::clear()
{
    data.clear();

    if(partitioned)
    {
        for(auto& node : nodes)
        {
            node.second.clear();
        }
    }
}

bool DataNode::operator<(const DataNode& rhs) const
{
    if(this == &rhs)
    {
        return false;
    }

    //If the max val is the same we want to deterministically choose one bin over the other
    if(getMaxVal().val == rhs.getMaxVal().val)
    {
        if(nodeLevel != rhs.nodeLevel)
        {
            return nodeLevel < rhs.nodeLevel;
        }

        return nodeIndex < rhs.nodeIndex;
    }
    

    return getMaxVal().val < rhs.getMaxVal().val;
}

bool DataNode::operator==(const DataNode& rhs) const
{
    return this == &rhs;
}


bool DataNode::PointerCompare::operator() (const DataNode* lhs, 
                                      const DataNode* rhs) const
{
    return *lhs < *rhs;
}

const PlumeDataEntry& DataNode::getMaxVal() const
{
    return maxVal;
}

const unsigned int DataNode::getNodeLevel()
{
    return nodeLevel;
}

bool DataNode::isPartitioned()
{
    return partitioned;
}