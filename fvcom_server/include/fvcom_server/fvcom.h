#ifndef FVCOM_H
#define FVCOM_H

#include <list>
#include <unordered_map>
#include <cstddef>
#include <stdexcept>
#include <memory>

#include <netcdf>


/**
 * Class used to load and query FVCOM data
 */
class FVCOM
{
public:
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
	FVCOM(std::string filename, int xChunkSize, int yChunkSize, int siglayChunkSize, int timeChunkSize);

private:

	/**
	 * Struct to group a (x, y, height) point
     */
	struct point
	{
		/**
		 * x location in meters
		 */
		float x;

		/**
		 * y location in meters
		 */
		float y;

		/**
		 * water depth (bathymetry) in meters
		 */
		float h;
	};

	/**
	 * Determines if a point is in the specified triangle
	 * @param testPoint Point to test with
	 * @param triangle Triangle to test with
	 */
	bool pointInTriangle(point testPoint, int triangle);

	/**
	 * Finds the triangle which contains the specified point
	 * @param testPoint Point to get containing triangle for
	 */
	int getContainingTriangle(point testPoint);

	/**
	 * Finds the closest node to a point
	 * @param testPoint Point to get the closest node for
	 */
	int getClosestNode(point testPoint);

	/**
	 * Gets the distance between two points
	 * @param p0 Point 0 for which to get the distance
	 * @param p1 Point 1 for which  to get the distance
	 */
	float distance(point p0, point p1);

	/**
	 * Helper function which loads all the model structure data from the model file
	 */
	void loadStructureData();

	/**
	 * Helper function that determines the extent of the model
	 */
	void getModelExtent();

	/**
	 * Helper function which splits the model into chunks which can be individually loaded
	 */
	void splitIntoChunks();

	/**
	 * Gets the chunk that contains the (node, sigma, time) tuple
	 * @param node Node to find the chunk for
	 * @param sigma Sigma layer to find the chunk for
	 * @param time Time slice to find the chunk for
	 * @return Chunk which contains the data for the specified node
	 */
	int getChunkForNode(int node, int siglay, int time);

	/**
	 * Gets the chunk that contains the (triangle, sigma, time) tuple
	 * @param triangle Triangle to find the chunk for
	 * @param sigma Sigma layer to find the chunk for
	 * @param time Time slice to find the chunk for
	 * @return Chunk which contains the data for the specified triangle
	 */
	int getChunkForTriangle(int triangle, int siglay, int time);



	/**
	 * Retrieves data from a specific node
	 * If the needed data is not loaded this function will call all the necessary functions to load it 
	 * @param node Node number of get data from
	 * @param sigma Sigma layer to get the data from
	 * @param time Time index to get the data from
	 */
	float getDataFromNode(int node, int sigma, int time);

	/**
	 * Retrieves data from a specific triangle
	 * If the needed data is not loaded this function will call all the necessary functions to load it 
	 * @param triangle Triangle number of get data from
	 * @param sigma Sigma layer to get the data from
	 * @param time Time index to get the data from
	 */

	float getDataFromTriangle(int triangle, int sigma, int time);

private:
	const netCDF::NcFile dataFile;

	/**
	 * Number of sigma layers
	 */
	unsigned int siglayDim;

	/**
	 * x,y for each node
	 */
	std::vector<point> nodes;

	/**
	 * x,y for each triangle
	 */
	std::vector<point> triangles;

	/**
	 * The times corresponding to each time index
	 */
	std::vector<float> times;

	/**
	 * List of nodes in each triangle
	 */
	std::vector<std::vector<int>> triangleToNodes;

	/**
	 * List of triangles that each node is a part of
	 */
	std::vector<std::vector<int>> nodeToTriangles;

	/**
	 * Chunk that a triangle is in
	 */
	std::vector<int> triangleToChunk;

	/**
	 * Chunk that a node is in
	 */
	std::vector<int> nodeToChunk;

	// X,Y extent of the model
	float minX;
	float minY;
	float maxX;
	float maxY;

	//Size of chunks in different dimensions
	int xChunkSize; //in meters
	int yChunkSize; //in meters
	int siglayChunkSize; //in siglay indices
	int timeChunkSize; //in time indicies

	//Number of chunks for each dimension
	int siglayDimChunks;
	int timeDimChunks;
	int yDimChunks;
	int xDimChunks;
};

#endif