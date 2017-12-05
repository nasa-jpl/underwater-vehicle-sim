#ifndef FVCOM_H
#define FVCOM_H

#include <list>
#include <unordered_map>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <netcdf>
#include <exception>

#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/LRUCache.h"


/**
 *Exception used when FVCOM is asked for something outside of the model extent
 */
struct FVCOMOutOfBounds : public std::exception {
	const char * what () const throw () {
		return "FVCOM Out of Bounds Exception";
	}
};

/**
 * Class used to load and query FVCOM data
 */
class FVCOM
{
public:

	struct FVCOMData
	{
		float u;
		float v;
		float temp;
		float salt;
		float dye;
	};
	
	/**
	 * Initalize FVCOM class with no data file,
     */
	FVCOM();

	/**
	 * Initalize FVCOM class with data from file,
     * @param filename File to load
     */
	FVCOM(std::string filename);

	/**
	 * Initalize FVCOM class with data from file and specify the size of the FVCOMChunks
     * @param filename File to load
     * @param xChunkSize Size of a chunk in the x direction
     * @param yChunkSize Size of a chunk in the y direction
     * @param siglayChunkSize Size of a chunk in the siglay direction
     * @param timeChunkSize Size of a chunk in the time direction
     */
	FVCOM(std::string filename, unsigned int xChunkSize, 
								unsigned int yChunkSize,
								unsigned int siglayChunkSize, 
								unsigned int timeChunkSize, 
								unsigned int cacheSize);


	const FVCOM::FVCOMData getData(float x, float y, float height, float time);

private:

	/**
	 * Retrieves data from a specific node
	 * If the needed data is not loaded this function will call all the necessary functions to load it 
	 * @param node Node number of get data from
	 * @param sigma Sigma layer to get the data from
	 * @param time Time index to get the data from
	 */
	float getDataForNode(int node, int sigma, int time);

	/**
	 * Retrieves data from a specific triangle
	 * If the needed data is not loaded this function will call all the necessary functions to load it 
	 * @param triangle Triangle number of get data from
	 * @param sigma Sigma layer to get the data from
	 * @param time Time index to get the data from
	 */
	float getDataForTriangle(int triangle, int sigma, int time);

private:
	
	FVCOMStructure structure;

	LRUCache<unsigned int, FVCOMChunk> chunkCache;

};

#endif