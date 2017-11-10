#ifndef FVCOM_STRUCTURE_H
#define FVCOM_STRUCTURE_H

#include <list>
#include <unordered_map>
#include <cstddef>
#include <stdexcept>
#include <memory>

#include <netcdf>


/**
 * Class used to load and query FVCOM structure data
 */
class FVCOMStructure
{
public:

	/**
	 * Initalize FVCOMStructure class with data from file and specify the size of the FVCOMChunks
     * @param filename File to load
     * @param xChunkSize Size of a chunk in the x direction
     * @param yChunkSize Size of a chunk in the y direction
     * @param siglayChunkSize Size of a chunk in the siglay direction
     * @param timeChunkSize Size of a chunk in the time direction
     */
	FVCOMStructure(const std::string filename, int xChunkSize, int yChunkSize, int siglayChunkSize, int timeChunkSize);

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

	struct ChunkInfo
	{
		unsigned int id;
		unsigned int xChunk;
		unsigned int yChunk;
		unsigned int siglayChunk;
		unsigned int timeChunk;

		unsigned int xStart;
		unsigned int yStart;
		unsigned int siglayStart;
		unsigned int timeStart;

		unsigned int xSize;
		unsigned int ySize;
		unsigned int siglaySize;
		unsigned int timeSize;
	};

	struct ModelFile
	{
		/**
		 * String of the filename for this file
		 */
		std::string filename;

		/**
		 * Start time for this file
		 */
		float startTime;

		/**
		 * Time index for the start time of this file
		 */
		unsigned int startTimeIndex;

		/**
		 * The Time dimension for this file
		 */
		unsigned int timeDim;

		bool operator<(const ModelFile& rhs) const { startTime < rhs.startTime; }
	};

	/**
	 * Determines if a point is in the specified triangle
	 * @param testPoint Point to test with
	 * @param triangle Triangle to test with
	 */
	bool pointInTriangle(point testPoint, int triangle) const;

	/**
	 * Finds the triangle which contains the specified point
	 * @param testPoint Point to get containing triangle for
	 */
	int getContainingTriangle(point testPoint) const;

	/**
	 * Finds the closest node to a point
	 * @param testPoint Point to get the closest node for
	 */
	int getClosestNode(point testPoint) const;

	/**
	 * Gets the time index that is closest to the given time
	 * @param time time to find the closest index for
	 * @return index for the closest time
	 */
	 int getClosestTime(float time) const;

	/**
	 * Gets the siglay that is closest to the given location
	 * @param testPoint location to find the closest siglay for
	 * @return index for the closest siglay
	 */
	int getClosestNodeSiglay(point testPoint) const;

	/**
	 * Gets the siglay that is closest to the given location
	 * @param testPoint location to find the closest siglay for
	 * @return index for the closest siglay
	 */
	int getClosestTriangleSiglay(point testPoint) const;

	/**
	 * Gets the distance between two points
	 * @param p0 Point 0 for which to get the distance
	 * @param p1 Point 1 for which  to get the distance
	 */
	float distance(point p0, point p1) const;

	/**
	 * Gets the chunk that contains the (node, sigma, time) tuple
	 * @param node Node to find the chunk for
	 * @param sigma Sigma layer to find the chunk for
	 * @param time Time slice to find the chunk for
	 * @return Chunk which contains the data for the specified node
	 */
	FVCOMStructure::ChunkInfo getChunkForNode(int node, int siglay, int time) const;

	/**
	 * Gets the chunk that contains the (triangle, sigma, time) tuple
	 * @param triangle Triangle to find the chunk for
	 * @param sigma Sigma layer to find the chunk for
	 * @param time Time slice to find the chunk for
	 * @return Chunk which contains the data for the specified triangle
	 */
	FVCOMStructure::ChunkInfo getChunkForTriangle(int triangle, int siglay, int time) const;

	const std::vector<unsigned int>& getNodesInChunk(FVCOMStructure::ChunkInfo chunk) const;
	const std::vector<unsigned int>& getTrianglesInChunk(FVCOMStructure::ChunkInfo chunk) const;

	const std::vector<ModelFile> getModelFiles() const;

	const bool pointInModel(point p, float time) const;
private:

	/**
	 * Loads all the data files
	 * @param filename Directory to load the files for
	 * @return List of filenames containing the model
	 */
	std::vector<std::string> traverseDataFiles(std::string filename);

	/**
	 * Helper function which loads all the model structure data from the model file
	 */
	void loadStructureData(const std::string filename);

	/**
	 * Helper function that determines the extent of the model
	 */
	void getModelExtent();

	/**
	 * Helper function which splits the model into chunks which can be individually loaded
	 */
	void splitIntoChunks();

private:

	/**
	 * File information for all the model files
	 */
	std::vector<ModelFile> modelFiles;

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
	 * siglay for nodes
	 */
	std::vector<std::vector<float>> nodeSiglay;

	/**
	 * siglay for triangles
	 */
	std::vector<std::vector<float>> triangleSiglay;

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

	/**
	 * A list of all the nodes in each chunk for loading
	 */
	std::vector<std::vector<unsigned int>> nodesInChunk;

	/**
	 * A list of all the triangles in each chunk for loading
	 */
	std::vector<std::vector<unsigned int>> trianglesInChunk;


	// X,Y extent of the model
	float minX;
	float minY;
	float maxX;
	float maxY;

	//Size of chunks in different dimensions
	unsigned int xChunkSize; //in meters
	unsigned int yChunkSize; //in meters
	unsigned int siglayChunkSize; //in siglay indices
	unsigned int timeChunkSize; //in time indicies

	//Number of chunks for each dimension
	unsigned int siglayDimChunks;
	unsigned int timeDimChunks;
	unsigned int yDimChunks;
	unsigned int xDimChunks;
};

#endif