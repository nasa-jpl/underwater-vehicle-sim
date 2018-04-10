#ifndef DATA_BINS_H
#define DATA_BINS_H

#include <vector>
#include <unordered_map>

#include "tf/LinearMath/Vector3.h"

#include "vent_planner/DataBin.h"
#include "plume_detector/PlumeData.h"

class DataBins
{

public:
    DataBins(const tf::Vector3 centerOrigin, const double sizeFromCenter, const double binSize);
    
    ~DataBins();

    /**
    * Adds data to the data bins. Creates bins as needed
    */
    void addData(const std::vector<PlumeData>& data);
    void addData(const PlumeData& data);

    /**
    *Gets a list of bins that have a local maxima in plume data max
    */
    const std::vector<std::reference_wrapper<DataBin>> getLocalMaxima();

    /**
    *Gets a list of bins that have a local maxima in plume data average
    */
    const std::vector<std::reference_wrapper<DataBin>> getLocalAverageMaxima();

    /**
    *Gets the neighbors of the given bin. Initalizes bins if needed.
    */
    const std::vector<std::reference_wrapper<DataBin>> getNeighbors(DataBin& dataBin);

    /**
    *Gets the size of the smallest bin at this location
    */
    const double getSmallestBinSize(const tf::Vector3& point);

    /**
    *Creates nested bins inside of the specified bin. Currently this does not re-distribute data to lower bins
    *@param dataBin Bin to partition
    *@param binSize Size of the bins in the partition
    */
    void partition(DataBin& dataBin, const double binSize);

private:
    DataBins(const tf::Vector3 centerOrigin, const double sizeFromCenter, const double binSize, const unsigned int binLevel);

    

    /**
    * Gets the index of the bin that a specific point is in
    */
    unsigned int toBinIndex(const tf::Vector3& point);
    
    /**
    *Gets the x,y coordinate of a specific bin
    */
    tf::Vector3 toXY(const unsigned int binIndex);

    /**
    *Gets the bin indicies for the bins neighboring the specified bin.
    *Both initalized and uninitalized bin indicies are returned
    */
    std::vector<unsigned int> getNeighborIndicies(const unsigned int binIndex);

    /**
    * Gets all the initalized bins neighboring the specified bin. Uninitalized bins are not returned
    */
    const std::vector<std::reference_wrapper<DataBin>> getInitalizedNeighbors(DataBin& dataBin);

    /**
    *Gets the number of initalized and uninitalized neighbors for this bin
    */
    unsigned int getNumNeighbors(DataBin& dataBin);

    /**
    * Gets a bin and creates it if it does not exist
    *@param point point to get the bin for
    *@return The bin for the given point
    */
    DataBin& createAndGetBin(const tf::Vector3& point);
    DataBin& createAndGetBin(const unsigned int binIndex);

    /**
    *Creates the bin with the specifed index if it does not already exist
    */
    void createBin(const unsigned int binIndex);


private:
    const tf::Vector3 centerOrigin;
    const tf::Vector3 binOrigin;

    const double binSize;
    const double sizeFromCenter;

    //The number of bins in a single dimension
    const double numBins;
    const unsigned int binLevel;

    std::unordered_map<unsigned int, DataBin> bins;
};

#endif