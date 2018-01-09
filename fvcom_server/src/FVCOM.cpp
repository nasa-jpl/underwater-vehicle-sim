#include "fvcom_server/FVCOM.h"
#include "ros/ros.h"

#include <stdexcept>
#include <math.h>

FVCOM::FVCOM() {}

FVCOM::FVCOM(std::string filename) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(10)),
	structure(FVCOMStructure(filename, 500, 500, 10, 10))
{}

FVCOM::FVCOM(std::string filename, unsigned int xChunkSize, unsigned int yChunkSize, unsigned int siglayChunkSize, unsigned int timeChunkSize, unsigned int cacheSize) :
	chunkCache(LRUCache<unsigned int, FVCOMChunk>(cacheSize)),
	structure(FVCOMStructure(filename, xChunkSize, yChunkSize, siglayChunkSize, timeChunkSize))
{}

FVCOM::FVCOMData FVCOM::interpolate(FVCOMStructure::point interpolatePoint, float time)
{	
	int time1Index, time2Index;
	double time1Percent;

	int siglay1Index, siglay2Index;
	double siglay1Percent;


	//Get indicies and ratio of time and siglay
	timeInterpolation(time, time1Index, time2Index, time1Percent);
	siglayInterpolation(interpolatePoint, siglay1Index, siglay2Index, siglay1Percent);


	//Interpolate X, Y
	FVCOMChunk::NodeData siglay1Time1Data; 
	FVCOMChunk::NodeData siglay1Time2Data;
	FVCOMChunk::NodeData siglay2Time1Data;
	FVCOMChunk::NodeData siglay2Time2Data;

	siglay1Time1Data = barycentricInterpolation(interpolatePoint, siglay1Index, time1Index);
	siglay1Time2Data = barycentricInterpolation(interpolatePoint, siglay1Index, time2Index);
	siglay2Time1Data = barycentricInterpolation(interpolatePoint, siglay2Index, time1Index);
	siglay2Time2Data = barycentricInterpolation(interpolatePoint, siglay2Index, time2Index);



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
	FVCOM::FVCOMData returnData;

	returnData.dye = siglay1Data.dye * siglay1Percent + siglay2Data.dye * (1 - siglay1Percent);
	returnData.temp = siglay1Data.temp * siglay1Percent + siglay2Data.temp * (1 - siglay1Percent);
	returnData.salt = siglay1Data.salt * siglay1Percent + siglay2Data.salt * (1 - siglay1Percent);


	//Get u,v. Not currently interpolated
	unsigned int triangle = structure.getContainingTriangle(interpolatePoint);
	unsigned int siglayTriangleIndex = structure.getClosestTriangleSiglay(interpolatePoint);
	unsigned int closestTimeIndex = structure.getClosestTime(time);
	const FVCOMChunk::TriangleData& triangleData = getTriangleData(triangle, siglayTriangleIndex, closestTimeIndex);
	returnData.u = triangleData.u;
	returnData.v = triangleData.v;

	return returnData;
}

void FVCOM::timeInterpolation(float time, int& time1Index, int& time2Index, double& time1Percent)
{
	time1Index = structure.getPreviousTimeIndex(time);
	float previousTime = structure.getTime(time1Index);

	//Time is exactly on a time division, no interpolation needed.
	if(previousTime == time)
	{
		time2Index = time1Index;
		time1Percent = 1;
	}
	else //time is spilt between divisions so it needs interpolation
	{
		float nextTime = structure.getTime(time2Index);
		time1Percent = (nextTime - time) / (nextTime - previousTime);
	}
}

void FVCOM::siglayInterpolation(FVCOMStructure::point& interpolatePoint, int& siglay1Index, int& siglay2Index, double& siglay1Percent)
{
	int containingTriangle = structure.getContainingTriangle(interpolatePoint);


	siglay1Index = siglay2Index = -1;

	double prevDot = 0;
	for(unsigned int i = 0; i < structure.getNumSiglays(); i++)
	{
		FVCOMStructure::Plane plane = structure.getTriangleSiglayPlane(containingTriangle, i);

		//calculate the dot product with the plane and the point to determine which side of the plane it is on
		double dot = plane.a * interpolatePoint.x + plane.b * interpolatePoint.y + plane.c * interpolatePoint.h + plane.d;

		if(i != 0)
		{
			if(dot == 0) //point is in the plane, use current siglayIndex and no interpolation needed
			{
				siglay1Index = i;
				siglay2Index = i;
				break;
			}
			else if(dot > 0 && prevDot < 0 || //The sign of dot has changed so the siglay has been found
				    dot < 0 && prevDot > 0)
			{
				siglay1Index = i - 1;
				siglay2Index = i;
				break;
			}
			else if(prevDot > 0 && prevDot < dot || //The distance from the plane to the point is increasing so we have passed it.
					prevDot < 0 && prevDot > dot)   //If this occurs then it means the point is above the 0th siglay, use the 0th siglay
			{
				siglay1Index = 0;
				siglay2Index = 0;
				break;
			}
		}

		prevDot = dot;
	}


	//The dot product was always decreasings however it never changed sign.
	//Therefore the point is below the final siglay, so use the final siglay.
	if(siglay1Index == siglay2Index && siglay2Index == -1)
	{
		siglay1Index = structure.getNumSiglays() -1;
		siglay2Index = structure.getNumSiglays() -1;
	}


	float upperH, lowerH;

	if(siglay1Index == siglay2Index) //The point is above the 0th siglay so and there is no data there
	{
		siglay1Percent = 1.0;
	}
	else
	{
		FVCOMStructure::Plane upperPlane = structure.getTriangleSiglayPlane(containingTriangle, siglay1Index);
		FVCOMStructure::Plane lowerPlane = structure.getTriangleSiglayPlane(containingTriangle, siglay2Index);

		float upperH = (-upperPlane.d - upperPlane.a * interpolatePoint.x - upperPlane.b * interpolatePoint.y) / upperPlane.c;
		float lowerH = (-lowerPlane.d - lowerPlane.a * interpolatePoint.x - lowerPlane.b * interpolatePoint.y) / lowerPlane.c;

		siglay1Percent = (lowerH - interpolatePoint.h) / (lowerH - upperH);
	}
	
}


FVCOMChunk::NodeData FVCOM::barycentricInterpolation(const FVCOMStructure::point& interpolatePoint, int siglayIndex, int timeIndex)
{
	FVCOMChunk::NodeData interpolatedData;
	int containingTriangle = structure.getContainingTriangle(interpolatePoint);
	const std::vector<int>& surroundingNodes = structure.getNodesInTriangle(containingTriangle);

	FVCOMStructure::point p1 = structure.getNodePoint(surroundingNodes[0]);
	FVCOMStructure::point p2 = structure.getNodePoint(surroundingNodes[1]);
	FVCOMStructure::point p3 = structure.getNodePoint(surroundingNodes[2]);

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

const double FVCOM::areaOfTriangle(const FVCOMStructure::point& p1, const FVCOMStructure::point& p2, const FVCOMStructure::point& p3)
{
	double a = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	double b = sqrt((p1.x - p3.x) * (p1.x - p3.x) + (p1.y - p3.y) * (p1.y - p3.y));
	double c = sqrt((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y));
	double s = (a + b + c) / 2;

	return sqrt(s * (s - a) * (s - b) * (s - c));
}

const FVCOM::FVCOMData FVCOM::getData(float x, float y, float height, float time)
{
	FVCOMStructure::point interpolatePoint;
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


const FVCOMChunk::NodeData& FVCOM::getNodeData(int node, int siglayNodeIndex, int timeIndex)
{
	FVCOMStructure::ChunkInfo nodeChunkInfo = structure.getChunkForNode(node, siglayNodeIndex, timeIndex);
	if(!chunkCache.exists(nodeChunkInfo.id))
	{
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(nodeChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(nodeChunkInfo);

		chunkCache.put(nodeChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, nodeChunkInfo));
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
		const std::vector<unsigned int>& nodesToLoad = structure.getNodesInChunk(triangleChunkInfo);
		const std::vector<unsigned int>& trianglesToLoad = structure.getTrianglesInChunk(triangleChunkInfo);

		chunkCache.put(triangleChunkInfo.id, FVCOMChunk(structure.getModelFiles(), nodesToLoad, trianglesToLoad, triangleChunkInfo));
	}

	FVCOMChunk& triangleChunk = chunkCache.get(triangleChunkInfo.id);
	const FVCOMChunk::TriangleData& triangleData = triangleChunk.getTriangleData(triangle, siglayTriangleIndex, timeIndex);

	return triangleData;
}