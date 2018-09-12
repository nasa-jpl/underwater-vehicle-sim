#include "vent_planner/DataBins.h"

#include <math.h>
#include <limits>
#include <stdexcept>

#include "vent_planner/DataBin.h"
#include "tf/LinearMath/Vector3.h"

#include "ros/ros.h"

DataBins::DataBins(const tf::Vector3 centerOrigin, const double sizeFromCenter, const double binSize) :
    centerOrigin(centerOrigin),
    sizeFromCenter(sizeFromCenter),
    binSize(binSize),
    binOrigin(centerOrigin.getX() - sizeFromCenter, centerOrigin.getY() - sizeFromCenter, centerOrigin.getZ()),
    numBins(ceil(sizeFromCenter / binSize) * 2),
    binLevel(0)
{
    if(numBins >= sqrt(std::numeric_limits<unsigned int>::max()))
    {
        throw std::out_of_range("DataBins: Too many bins needed");
    }
}

DataBins::DataBins(const tf::Vector3 centerOrigin, const double sizeFromCenter, const double binSize, const unsigned int binLevel) :
    centerOrigin(centerOrigin),
    sizeFromCenter(sizeFromCenter),
    binSize(binSize),
    binOrigin(centerOrigin.getX() - sizeFromCenter, centerOrigin.getY() - sizeFromCenter, centerOrigin.getZ()),
    numBins(ceil(sizeFromCenter / binSize) * 2),
    binLevel(binLevel)
{
    if(numBins >= sqrt(std::numeric_limits<unsigned int>::max()))
    {
        throw std::out_of_range("DataBins: Too many bins needed");
    }
}

DataBins::~DataBins() {}

const std::vector<std::reference_wrapper<DataBin>> DataBins::getLocalMaxima()
{
    ROS_DEBUG("Data bins local maxima. Num Bins: %lu, Bin Level: %i", bins.size(), binLevel);
    std::vector<std::reference_wrapper<DataBin>> maxima;
    for (auto& it : bins) 
    {
        if(it.second.nestedBins)
        {
            ROS_DEBUG("Call data bins nested local maxima");
            std::vector<std::reference_wrapper<DataBin>> nestedMaxima = it.second.nestedBins->getLocalMaxima();
            maxima.insert(maxima.end(), nestedMaxima.begin(), nestedMaxima.end());
        }
        //Get all the neighbors for this bin
        std::vector<std::reference_wrapper<DataBin>> neighbors = getInitalizedNeighbors(it.second);

        //check to see if all the neighbors have a lower max val than this one
        bool maximum = true;
        if(neighbors.size() == getNumNeighbors(it.second))
        {
            for(auto neighbor : neighbors)
            {
                if(it.second.getMaxVal() < neighbor.get().getMaxVal())
                {
                    maximum = false;
                    break;
                }
            }  
        }
        else
        {
            maximum = false;
        }

        if(maximum)
        {
            ROS_DEBUG("DataBins max found, %p, Val: %f X: %f Y: %f BinLevel: %i neighbors.size(): %lu, Num Neighbors: %i", (void*)&(it.second), 
                                                                                it.second.getMaxVal(), 
                                                                                it.second.getCenterLocation().getX(), 
                                                                                it.second.getCenterLocation().getY(), 
                                                                                it.second.getBinLevel(),
                                                                                neighbors.size(),
                                                                                getNumNeighbors(it.second));
            maxima.push_back(it.second);
        }
    }

    return maxima;
}

const std::vector<std::reference_wrapper<DataBin>> DataBins::getLocalAverageMaxima()
{
    std::vector<std::reference_wrapper<DataBin>> maxima;

    for (auto& it : bins) 
    {
        if(it.second.nestedBins)
        {
            std::vector<std::reference_wrapper<DataBin>> nestedMaxima = it.second.nestedBins->getLocalAverageMaxima();
            maxima.insert(maxima.end(), nestedMaxima.begin(), nestedMaxima.end());
        }
        //Get all the neighbors for this bin
        std::vector<std::reference_wrapper<DataBin>> neighbors = getInitalizedNeighbors(it.second);

        //check to see if all the neighbors have a lower max val than this one
        bool maximum = true;
        if(neighbors.size() == getNumNeighbors(it.second))
        {
            for(auto neighbor : neighbors)
            {
                if(it.second.getAverage() < neighbor.get().getAverage())
                {
                    maximum = false;
                    break;
                }
            }  
        }
        else
        {
            maximum = false;
        }

        if(maximum)
        {
            maxima.push_back(it.second);
        }
    }

    return maxima;
}

const std::vector<std::reference_wrapper<DataBin>> DataBins::getInitalizedNeighbors(DataBin& dataBin)
{
    if(dataBin.getBinLevel() == binLevel)
    {
       unsigned int binIndex = toBinIndex(dataBin.getCenterLocation());
        std::vector<std::reference_wrapper<DataBin>> neighbors;

        std::vector<unsigned int> neighborIndicies = getNeighborIndicies(binIndex);

        for(unsigned int index : neighborIndicies)
        {
            auto bin = bins.find(index);
            if(bin != bins.end())
            {
                neighbors.push_back(bin->second);
            }
        } 
        return neighbors;
    }

    if(dataBin.nestedBins)
    {
        return dataBin.nestedBins->getInitalizedNeighbors(dataBin);
    }

    std::vector<std::reference_wrapper<DataBin>> empty;
    return empty;
}

const std::vector<std::reference_wrapper<DataBin>> DataBins::getNeighbors(DataBin& dataBin)
{
    if(dataBin.getBinLevel() == binLevel)
    {
        std::vector<std::reference_wrapper<DataBin>> neighbors;
        unsigned int binIndex = toBinIndex(dataBin.getCenterLocation());

        std::vector<unsigned int> neighborIndicies = getNeighborIndicies(binIndex);

        for(unsigned int index : neighborIndicies)
        {
            //Create bin if it is not already created
            DataBin& bin = createAndGetBin(index);
            neighbors.push_back(bin);
        }
        return neighbors;
    }
    
    if(dataBin.nestedBins)
    {
        return dataBin.nestedBins->getNeighbors(dataBin);
    }

    std::vector<std::reference_wrapper<DataBin>> empty;
    return empty;
}

void DataBins::partition(DataBin& dataBin, const double binSize)
{
    dataBin.nestedBins = std::unique_ptr<DataBins>(new DataBins(dataBin.centerLocation, 
                                                                dataBin.size / 2, 
                                                                binSize, 
                                                                binLevel + 1));
}

void DataBins::addData(const PlumeDataEntry& data)
{
    tf::Vector3 dataVector(data.x, data.y, data.h);
    DataBin& bin = createAndGetBin(dataVector);
    if(bin.nestedBins)
    {
        bin.nestedBins->addData(data);
    }
    else
    {
        bin.addData(data);
    }
    
}

void DataBins::addData(const std::vector<PlumeDataEntry>& data)
{
    for(const PlumeDataEntry& entry : data)
    {
        addData(entry);
    }
}

DataBin& DataBins::createAndGetBin(const tf::Vector3& point)
{
    unsigned int binIndex = toBinIndex(point);
    if(bins.count(binIndex) == 0)
    {
        createBin(binIndex);
    }
    
    auto returnBin = bins.find(binIndex);
    return returnBin->second;
}

DataBin& DataBins::createAndGetBin(const unsigned int binIndex)
{
    if(bins.count(binIndex) == 0)
    {
        createBin(binIndex);
    }
    
    auto returnBin = bins.find(binIndex);
    return returnBin->second;
}

const double DataBins::getSmallestBinSize(const tf::Vector3& point)
{
    unsigned int binIndex = toBinIndex(point);
    auto bin = bins.find(binIndex);
    if(bin != bins.end())
    {
        //If there are nested bins recursivly call smallest bin size
        if(bin->second.nestedBins)
        {
            return bin->second.nestedBins->getSmallestBinSize(point);
        }
    }

    //return this bin size if there are no more nested bins
    return binSize;
}

unsigned int DataBins::toBinIndex(const tf::Vector3& point)
{
    if(fabs(point.getX() - centerOrigin.getX()) >= sizeFromCenter ||
       fabs(point.getY() - centerOrigin.getY()) >= sizeFromCenter)
    {
        throw std::out_of_range("DataBins: Point out of range");
    }
    unsigned int xBin = (point.getX() - binOrigin.getX()) / binSize;
    unsigned int yBin = (point.getY() - binOrigin.getY()) / binSize;

    return xBin * numBins + yBin;
}

tf::Vector3 DataBins::toXY(const unsigned int binIndex)
{
    unsigned int xBin = binIndex / numBins;
    unsigned int yBin = binIndex - (xBin * numBins);
 
    if(xBin >= numBins ||
       yBin >= numBins)
    {
        throw std::out_of_range("DataBins: Bin out of range");
    }

    tf::Vector3 vec(binOrigin.getX() + xBin * binSize, binOrigin.getY() + yBin * binSize, binOrigin.getZ());
    return vec;
}

unsigned int DataBins::getNumNeighbors(DataBin& dataBin)
{
    
    if(dataBin.getBinLevel() == binLevel)
    {
        unsigned int binIndex = toBinIndex(dataBin.getCenterLocation());
        return getNeighborIndicies(binIndex).size();
    }
    
    if(dataBin.nestedBins)
    {
        return dataBin.nestedBins->getNumNeighbors(dataBin);
    }

    return 0;
}

std::vector<unsigned int> DataBins::getNeighborIndicies(const unsigned int binNum)
{
    std::vector<unsigned int> neighbors;

    unsigned int xBin = binNum / numBins;
    unsigned int yBin = binNum - (xBin * numBins);

    for(int xOffset = -1; xOffset <= 1; xOffset++)
    {
        for(int yOffset = -1; yOffset <= 1; yOffset++)
        {
            if(!(xOffset == 0 && yOffset == 0) &&
               !(xBin == 0 && xOffset == -1) &&
               !(xBin == numBins - 1 && xOffset == 1) &&
               !(yBin == 0 && yOffset == -1) &&
               !(yBin == numBins - 1 && yOffset == 1))
            {
                neighbors.push_back((xBin + xOffset) * numBins + (yBin + yOffset));
            }
        }
    }

    return neighbors;
}

void DataBins::createBin(const unsigned int binNum)
{
    tf::Vector3 centerLocation = toXY(binNum);
    centerLocation.setX(centerLocation.getX() + binSize / 2);
    centerLocation.setY(centerLocation.getY() + binSize / 2);
    if(bins.count(binNum) == 0)
    {
        DataBin bin(centerLocation, binSize, binLevel, binNum);
        bins.insert(std::make_pair(binNum, std::move(bin))); //must be moved as Bin has a unique_ptr
    }
}

