#include "fvcom_server/FVCOM.h"

FVCOM::FVCOM() {}

FVCOM::FVCOM(std::string filename) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(10)),
	structure(FVCOMStructure(filename, 500, 500, 10, 10))
{}

FVCOM::FVCOM(std::string filename, unsigned int xChunkSize, unsigned int yChunkSize, unsigned int siglayChunkSize, unsigned int timeChunkSize, unsigned int cacheSize) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(cacheSize)),
	structure(FVCOMStructure(filename, xChunkSize, yChunkSize, siglayChunkSize, timeChunkSize))
{}

const FVCOM::FVCOMData FVCOM::getData(float x, float y, float height, float time)
{
	FVCOMStructure::point p;
	p.x = x;
	p.y = y;
	p.h = height;

	//Throw an exception if the requested point is outside of the model extent
	if(!structure.pointInModel(p, time))
	{
		throw FVCOMOutOfBounds();
	}

	unsigned int node = structure.getClosestNode(p);
	unsigned int triangle = structure.getContainingTriangle(p);

	unsigned int siglayNodeIndex = structure.getClosestNodeSiglay(p);
	unsigned int siglayTriangleIndex = structure.getClosestTriangleSiglay(p);
	unsigned int timeIndex = structure.getClosestTime(time); 

	//Get the chunk info for the need points
	FVCOMStructure::ChunkInfo nodeChunkInfo = structure.getChunkForNode(node, siglayNodeIndex, timeIndex);
	FVCOMStructure::ChunkInfo triangleChunkInfo = structure.getChunkForTriangle(triangle, siglayTriangleIndex, timeIndex);

	//Load chunks if they do not exist
	if(!chunkCache.exists(nodeChunkInfo.id))
	{
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(nodeChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(nodeChunkInfo);

		chunkCache.put(nodeChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, nodeChunkInfo));
	}

	if(!chunkCache.exists(triangleChunkInfo.id))
	{
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(triangleChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(triangleChunkInfo);

		chunkCache.put(triangleChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, triangleChunkInfo));
	}

	//Get chunks from cache
	FVCOMChunk& nodeChunk = chunkCache.get(nodeChunkInfo.id);
	FVCOMChunk& triangleChunk = chunkCache.get(triangleChunkInfo.id);

	//Get data from Chunks
	const FVCOMChunk::NodeData& nodeData = nodeChunk.getNodeData(node, siglayNodeIndex, timeIndex);
	const FVCOMChunk::TriangleData& triangleData = triangleChunk.getTriangleData(triangle, siglayTriangleIndex, timeIndex);

	FVCOM::FVCOMData returnData;
	returnData.temp = nodeData.temp;
	returnData.salt = nodeData.salt;
	returnData.dye = nodeData.dye;

	//ISSUE HERE
	returnData.u = triangleData.u;
	returnData.v = triangleData.v;

	return returnData;
}