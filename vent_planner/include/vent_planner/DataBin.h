#ifndef DATA_BIN_H
#define DATA_BIN_H

#include <vector>
#include <memory>


#include "tf/LinearMath/Vector3.h"
#include "plume_detector/PlumeData.h"

class DataBins;

class DataBin
{
public:
    friend class DataBins;

    DataBin(tf::Vector3 centerLocation, const double size, const unsigned int binLevel);
    
    void addData(const PlumeData& newData);

    const std::vector<PlumeData>& getData();
    const double getMaxVal() const;
    const tf::Vector3& getMaxValLocation();
    const double getAverage();
    const double getHeightOfPlume();

    const tf::Vector3& getCenterLocation();
    const unsigned int getBinLevel();
    const double getSize() const;

    void clear();

    bool operator<(const DataBin& rhs) const;
    bool operator==(const DataBin& rhs) const;
    struct RefCompare
    {
        bool operator() (const std::reference_wrapper<DataBin> lhs, const std::reference_wrapper<DataBin> rhs) const;
    };

    const bool isPartitioned() const;
private:
    std::vector<PlumeData> data;
    std::unique_ptr<DataBins> nestedBins;
    const tf::Vector3 centerLocation;
    const double size;
    const unsigned int binLevel;
    double maxVal;
    tf::Vector3 maxValLocation;
    double average;
};

#endif