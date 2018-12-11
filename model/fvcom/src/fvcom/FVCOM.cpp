#include "fvcom/FVCOM.h"
#include "model_interface/ModelData.h"

#include <stdexcept>
#include <math.h>
#include  <limits>

FVCOM::FVCOM() {}

FVCOM::FVCOM(std::string filename) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(100)),
	structure(FVCOMStructure(filename, 2000, 2000, 100, 10)),
	startLoad(nullptr),
	endLoad(nullptr)
{}

FVCOM::FVCOM(std::string filename, void (*startLoad)(void), void (*endLoad)(void)) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(100)),
	structure(FVCOMStructure(filename, 2000, 2000, 100, 10)),
	startLoad(startLoad),
	endLoad(endLoad)
{

}

FVCOM::FVCOM(std::string filename, unsigned int xChunkSize, unsigned int yChunkSize, unsigned int siglayChunkSize, unsigned int timeChunkSize, unsigned int cacheSize) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(cacheSize)),
	structure(FVCOMStructure(filename, xChunkSize, yChunkSize, siglayChunkSize, timeChunkSize)),
	startLoad(nullptr),
	endLoad(nullptr)
{}

FVCOM::FVCOM(std::string filename,
			 void (*startLoad)(void),
			 void (*endLoad)(void),
			 unsigned int xChunkSize,
			 unsigned int yChunkSize,
			 unsigned int siglayChunkSize,
			 unsigned int timeChunkSize,
			 unsigned int cacheSize) :
		chunkCache(LRUCache<unsigned int, FVCOMChunk>(cacheSize)),
		structure(FVCOMStructure(filename, xChunkSize, yChunkSize, siglayChunkSize, timeChunkSize)),
		startLoad(startLoad),
		endLoad(endLoad)
{}

ModelData FVCOM::interpolate(FVCOMStructure::Point interpolatePoint, float time)
{	
	int time1Index, time2Index;
	double time1Percent;

	int siglay1Index, siglay2Index;
	double siglay1Percent;

	int containingTriangle = structure.getContainingTriangle(interpolatePoint);
	
	//Get indicies and ratio of time and siglay
	structure.timeInterpolation(time, time1Index, time2Index, time1Percent);
	structure.siglayInterpolation(interpolatePoint, siglay1Index, siglay2Index, siglay1Percent, containingTriangle);


	//Interpolate X, Y
	FVCOMChunk::NodeData siglay1Time1Data; 
	FVCOMChunk::NodeData siglay1Time2Data;
	FVCOMChunk::NodeData siglay2Time1Data;
	FVCOMChunk::NodeData siglay2Time2Data;

	

	siglay1Time1Data = barycentricInterpolation(interpolatePoint, containingTriangle, siglay1Index, time1Index);
	siglay1Time2Data = barycentricInterpolation(interpolatePoint, containingTriangle, siglay1Index, time2Index);
	siglay2Time1Data = barycentricInterpolation(interpolatePoint, containingTriangle, siglay2Index, time1Index);
	siglay2Time2Data = barycentricInterpolation(interpolatePoint, containingTriangle, siglay2Index, time2Index);



	//Interpolate time
	FVCOMChunk::NodeData siglay1Data;
	FVCOMChunk::NodeData siglay2Data;
	
	siglay1Data.dye = siglay1Time1Data.dye * time1Percent + siglay1Time2Data.dye * (1 - time1Percent);
	siglay1Data.temp = siglay1Time1Data.temp * time1Percent + siglay1Time2Data.temp * (1 - time1Percent);
	siglay1Data.salt = siglay1Time1Data.salt * time1Percent + siglay1Time2Data.salt * (1 - time1Percent);

	siglay2Data.dye = siglay2Time1Data.dye * time1Percent + siglay2Time2Data.dye * (1 - time1Percent);
	siglay2Data.temp = siglay2Time1Data.temp * time1Percent + siglay2Time2Data.temp * (1 - time1Percent);
	siglay2Data.salt = siglay2Time1Data.salt * time1Percent + siglay2Time2Data.salt * (1 - time1Percent);


	//Interpolate siglay
	ModelData returnData;

	returnData.dye = siglay1Data.dye * siglay1Percent + siglay2Data.dye * (1 - siglay1Percent);
	returnData.temp = siglay1Data.temp * siglay1Percent + siglay2Data.temp * (1 - siglay1Percent);
	returnData.salt = siglay1Data.salt * siglay1Percent + siglay2Data.salt * (1 - siglay1Percent);


	//Get u,v. Not currently interpolated
	unsigned int siglayTriangleIndex = structure.getClosestTriangleSiglay(interpolatePoint, containingTriangle);
	unsigned int closestTimeIndex = structure.getClosestTime(time);
	const FVCOMChunk::TriangleData& triangleData = getTriangleData(containingTriangle, siglayTriangleIndex, closestTimeIndex);
	returnData.u = triangleData.u;
	returnData.v = triangleData.v;

	//Get depth
	returnData.depth = structure.getDepthAtPoint(interpolatePoint, containingTriangle);
	return returnData;
}

FVCOMChunk::NodeData FVCOM::barycentricInterpolation(const FVCOMStructure::Point& interpolatePoint, int containingTriangle, int siglayIndex, int timeIndex)
{
	FVCOMChunk::NodeData interpolatedData;
	const std::vector<int>& surroundingNodes = structure.getNodesInTriangle(containingTriangle);

	FVCOMStructure::Point p1 = structure.getNodePoint(surroundingNodes[0]);
	FVCOMStructure::Point p2 = structure.getNodePoint(surroundingNodes[1]);
	FVCOMStructure::Point p3 = structure.getNodePoint(surroundingNodes[2]);

	const FVCOMChunk::NodeData& p1Data = getNodeData(surroundingNodes[0], siglayIndex, timeIndex);
	const FVCOMChunk::NodeData& p2Data = getNodeData(surroundingNodes[1], siglayIndex, timeIndex);
	const FVCOMChunk::NodeData& p3Data = getNodeData(surroundingNodes[2], siglayIndex, timeIndex);

	double totalArea = areaOfTriangle(p1, p2, p3);

	//Calculate the percentage of each point
	double point1Percent = areaOfTriangle(interpolatePoint, p2, p3) / totalArea;
	double point2Percent = areaOfTriangle(interpolatePoint, p1, p3) / totalArea;
	double point3Percent = areaOfTriangle(interpolatePoint, p1, p2) / totalArea;

	//Set data to be returned
	interpolatedData.temp = p1Data.temp * point1Percent + p2Data.temp * point2Percent + p3Data.temp * point3Percent;
	interpolatedData.salt = p1Data.salt * point1Percent + p2Data.salt * point2Percent + p3Data.salt * point3Percent;
	interpolatedData.dye = p1Data.dye * point1Percent + p2Data.dye * point2Percent + p3Data.dye * point3Percent;

	return interpolatedData;
}

const double FVCOM::areaOfTriangle(const FVCOMStructure::Point& p1, const FVCOMStructure::Point& p2, const FVCOMStructure::Point& p3) const
{
	double a = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	double b = sqrt((p1.x - p3.x) * (p1.x - p3.x) + (p1.y - p3.y) * (p1.y - p3.y));
	double c = sqrt((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y));
	double s = (a + b + c) / 2;

	return sqrt(s * (s - a) * (s - b) * (s - c));
}

const ModelData FVCOM::getData(double x, double y, double height, double time)
{
	FVCOMStructure::Point interpolatePoint;
	interpolatePoint.x = x;
	interpolatePoint.y = y;
	interpolatePoint.h = height;

	//Throw an exception if the requested point is outside of the model extent
	if(!structure.pointInModel(interpolatePoint, time))
	{
		throw std::out_of_range("FVCOM request outside of model extent");
	}

	return interpolate(interpolatePoint, time);
}

const ModelData FVCOM::getDataOutOfRange(double x, double y, double height, double time)
{
	//if out of range XY then get closest node
	//if out of range time then get closest time

	FVCOMStructure::Point interpolatePoint;
	interpolatePoint.x = x;
	interpolatePoint.y = y;
	interpolatePoint.h = height;

	ModelData data;
	if(!structure.xyInModel(interpolatePoint))
	{
		int node = structure.getClosestNode(interpolatePoint);
		FVCOMStructure::Point nodePoint = structure.getNodePoint(node);

		data.depth = nodePoint.h;
		data.u = std::numeric_limits<double>::quiet_NaN();
		data.v = std::numeric_limits<double>::quiet_NaN();

		data.salt = std::numeric_limits<double>::quiet_NaN();
		data.temp = std::numeric_limits<double>::quiet_NaN();
		data.dye = 0;
	}
	else if(!structure.timeInModel(time))
	{
		data.depth = structure.getDepthAtPoint(interpolatePoint);
		data.u = std::numeric_limits<double>::quiet_NaN();
		data.v = std::numeric_limits<double>::quiet_NaN();

		data.salt = std::numeric_limits<double>::quiet_NaN();
		data.temp = std::numeric_limits<double>::quiet_NaN();
		data.dye = 0;
	}
	else if(!structure.depthInModel(interpolatePoint))
	{
		data.depth = structure.getDepthAtPoint(interpolatePoint);
		data.u = std::numeric_limits<double>::quiet_NaN();
		data.v = std::numeric_limits<double>::quiet_NaN();

		data.salt = std::numeric_limits<double>::quiet_NaN();
		data.temp = std::numeric_limits<double>::quiet_NaN();
		data.dye = std::numeric_limits<double>::quiet_NaN();
	}
	
	return data;
}

const FVCOMChunk::NodeData& FVCOM::getNodeData(int node, int siglayNodeIndex, int timeIndex)
{
	FVCOMStructure::ChunkInfo nodeChunkInfo = structure.getChunkForNode(node, siglayNodeIndex, timeIndex);
	if(!chunkCache.exists(nodeChunkInfo.id))
	{
		if(startLoad)
		{
			startLoad();
		}
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(nodeChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(nodeChunkInfo);

		chunkCache.put(nodeChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, nodeChunkInfo));
		if(endLoad)
		{
			endLoad();
		}
	}

	FVCOMChunk& nodeChunk = chunkCache.get(nodeChunkInfo.id);
	const FVCOMChunk::NodeData& nodeData = nodeChunk.getNodeData(node, siglayNodeIndex, timeIndex);

	return nodeData;
}

const FVCOMChunk::TriangleData& FVCOM::getTriangleData(int triangle, int siglayTriangleIndex, int timeIndex)
{
	FVCOMStructure::ChunkInfo triangleChunkInfo = structure.getChunkForTriangle(triangle, siglayTriangleIndex, timeIndex);
	if(!chunkCache.exists(triangleChunkInfo.id))
	{
		if(startLoad)
		{
			startLoad();
		}
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(triangleChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(triangleChunkInfo);

		chunkCache.put(triangleChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, triangleChunkInfo));
		if(endLoad)
		{
			endLoad();
		}
	}

	FVCOMChunk& triangleChunk = chunkCache.get(triangleChunkInfo.id);
	const FVCOMChunk::TriangleData& triangleData = triangleChunk.getTriangleData(triangle, siglayTriangleIndex, timeIndex);

	return triangleData;
}
