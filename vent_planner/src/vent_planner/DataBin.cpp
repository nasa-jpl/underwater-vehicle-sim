#include "vent_planner/DataBin.h"

#include <limits>

#include "vent_planner/DataBins.h"

#include "tf/LinearMath/Vector3.h"

DataBin::DataBin(tf::Vector3 centerLocation, const double size, const unsigned int binLevel) :
    centerLocation(centerLocation),
    size(size),
    maxVal(-std::numeric_limits<double>::max()),
    maxValLocation(0,0,0),
    average(0),
    binLevel(binLevel)
{}

void DataBin::addData(const PlumeData& newData)
{
    average = (average * data.size() + newData.val) / (data.size() + 1);
    data.push_back(newData);
    if(newData.val > maxVal)
    {
        maxVal = newData.val;
        maxValLocation.setX(newData.x);
        maxValLocation.setY(newData.y);
        maxValLocation.setZ(newData.h);
    }

}

const std::vector<PlumeData>& DataBin::getData()
{
    return data;
}

const tf::Vector3& DataBin::getCenterLocation()
{
    return centerLocation;
}

const unsigned int DataBin::getBinLevel()
{
    return binLevel;
}

const double DataBin::getMaxVal() const
{
    return maxVal;
}

const tf::Vector3& DataBin::getMaxValLocation()
{
    return maxValLocation;
}

const double DataBin::getAverage()
{
    return average;
}

const double DataBin::getSize() const
{
    return size;
}

void DataBin::clear()
{
    data.clear();
    average = 0;
    maxVal = 0;
}

bool DataBin::operator<(const DataBin& rhs) const
{
    if(this == &rhs)
    {
        return false;
    }

    if(getMaxVal() == rhs.getMaxVal())
    {
        return this < &rhs;
    }

    return getMaxVal() < rhs.getMaxVal();
}

bool DataBin::operator==(const DataBin& rhs) const
{
    return this == &rhs;
}


bool DataBin::RefCompare::operator() (const std::reference_wrapper<DataBin> lhs, 
                                      const std::reference_wrapper<DataBin> rhs) const
{
    return lhs.get() < rhs.get();
}

const bool DataBin::isPartitioned() const
{
    return nestedBins != 0;
}

const double DataBin::getHeightOfPlume()
{
    //bin data by depth return bin with largest average
    unsigned int binSize = 10;
    double minHeight = std::numeric_limits<double>::max();
    double maxHeight = -std::numeric_limits<double>::max();

    //if there is no data then there is no plume height to find
    if(data.size() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    //calculate min and max heights for bins
    for(unsigned int i = 0; i < data.size(); i++)
    {
        auto& d = data[i];
        if(minHeight > d.h)
        {
            minHeight = d.h;
        }

        if(maxHeight < d.h)
        {
            maxHeight = d.h;
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

    std::vector<double> bins(numBins, 0);
    std::vector<int> binCount(numBins, 0);

    for(unsigned int i = 0; i < data.size(); i++)
    {
        auto& d = data[i];
        int bin = (d.h - minHeight) / binSize;
        bins[bin] += d.val;
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