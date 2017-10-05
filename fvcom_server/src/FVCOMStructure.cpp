#include "fvcom_server/FVCOMStructure.h"

#include <netcdf>
#include <memory>
#include <cmath>
#include <limits>


FVCOMStructure::FVCOMStructure(const netCDF::NcFile dataFile, int xChunkSize, int yChunkSize, int siglayChunkSize, int timeChunkSize) :
	xChunkSize(xChunkSize),
	yChunkSize(yChunkSize),
	siglayChunkSize(siglayChunkSize),
	timeChunkSize(timeChunkSize)
{
	loadStructureData(dataFile);
	splitIntoChunks();
}

void FVCOMStructure::loadStructureData(const netCDF::NcFile dataFile)
{
	//Get dimensions of structure elements
	unsigned int nodeDim = dataFile.getDim("node").getSize();
	unsigned int neleDim = dataFile.getDim("nele").getSize();
	unsigned int timeDim = dataFile.getDim("time").getSize();
	siglayDim = dataFile.getDim("siglay").getSize();

	//Load all variables for the structure of the model
	netCDF::NcVar xVar = dataFile.getVar("x");
	netCDF::NcVar yVar = dataFile.getVar("y");
	netCDF::NcVar xcVar = dataFile.getVar("xc");
	netCDF::NcVar ycVar = dataFile.getVar("yc");
	netCDF::NcVar nvVar = dataFile.getVar("nv");
	netCDF::NcVar hVar = dataFile.getVar("h");
	netCDF::NcVar centerHVar = dataFile.getVar("center_h");
	netCDF::NcVar timeVar = dataFile.getVar("time");

	std::vector<float> nodeX;
	std::vector<float> nodeY;
	std::vector<float> nodeH;

	std::vector<float> triangleX;
	std::vector<float> triangleY;
	std::vector<float> triangleH;
	

	nodeX.resize(nodeDim);
	nodeY.resize(nodeDim);
	triangleX.resize(neleDim);
	triangleY.resize(neleDim);
	nodeH.resize(nodeDim);
	triangleH.resize(neleDim);
	times.resize(timeDim);

	//resize for multidimensional array
	triangleToNodes.resize(3);
	for(int i = 0; i < 3; i++)
	{
		triangleToNodes[i].resize(neleDim);
	}


	//Assign all arrays for the structure variables
	xVar.getVar(nodeX.data());
	yVar.getVar(nodeY.data());
	xcVar.getVar(triangleX.data());
	ycVar.getVar(triangleY.data());
	hVar.getVar(nodeH.data());
	centerHVar.getVar(triangleH.data());
	timeVar.getVar(times.data());

	//load nvVar into a multidimensional vector
	for(unsigned int i = 0; i < 3; i++)
	{
		std::vector<size_t> start = {i, 0};
		std::vector<size_t> count = {1, neleDim};
		nvVar.getVar(start, count, triangleToNodes[i].data());
	}

	//Convert to use point struct
	nodes.resize(nodeDim);
	triangles.resize(neleDim);
	for(int i = 0; i < nodeDim; i++)
	{
		nodes[i].x = nodeX[i];
		nodes[i].y = nodeY[i];
		nodes[i].h = nodeH[i];
	}

	for(int i = 0; i < neleDim; i++)
	{
		triangles[i].x = triangleX[i];
		triangles[i].y = triangleY[i];
		triangles[i].h = triangleH[i];
	}


	//The nv variable from the netCDF indexes starting at 1
	//Convert this to 0 by subtracting 1 from every value
	for(unsigned int i = 0; i < neleDim; i++)
	{
		for(unsigned int j = 0; j < 3; j++)
		{
			triangleToNodes[j][i]--;
		}
	}
	
	//Pre Processes model to get node to triangle conversion
	nodeToTriangles.resize(nodeDim);

	for(unsigned int i = 0; i < neleDim; i++)
	{
		for(unsigned int j = 0; j < 3; j++)
		{
			int triangle = i;
			int node = triangleToNodes[j][i];

			nodeToTriangles[node].push_back(triangle);
		}
	}
}

void FVCOMStructure::splitIntoChunks()
{
	getModelExtent();

	siglayDimChunks = std::ceil(siglayDim / (double)siglayChunkSize);
	timeDimChunks = std::ceil(times.size() / (double)timeChunkSize);
	yDimChunks = std::ceil((maxY - minY) / (double)yChunkSize);
	xDimChunks = std::ceil((maxX - minX) / (double)xChunkSize);
}

void FVCOMStructure::getModelExtent()
{
	for(int i = 0; i < nodes.size(); i++)
	{
		point node = nodes[i];
		
		if(node.x > maxX)
		{
			maxX = node.x;
		}

		if(node.x < minX)
		{
			minX = node.x;
		}

		if(node.y > maxY)
		{
			maxY = node.y;
		}

		if(node.y < minY)
		{
			minY = node.y;
		}
	}
}

bool FVCOMStructure::pointInTriangle(point testPoint, int triangle)
{
	int p0Index = triangleToNodes[0][triangle];
	int p1Index = triangleToNodes[1][triangle];
	int p2Index = triangleToNodes[2][triangle];

	point p0 = nodes[p0Index];
	point p1 = nodes[p1Index];
	point p2 = nodes[p2Index];

	//Calculate barycentric coordinates
	float alpha = ((p1.y - p2.y)*(testPoint.x - p2.x) + (p2.x - p1.x)*(testPoint.y - p2.y)) /
        ((p1.y - p2.y)*(p0.x - p2.x) + (p2.x - p1.x)*(p0.y - p2.y));

	float beta = ((p2.y - p0.y)*(testPoint.x - p2.x) + (p0.x - p2.x)*(testPoint.y - p2.y)) /
       	((p1.y - p2.y)*(p0.x - p2.x) + (p2.x - p1.x)*(p0.y - p2.y));

	float gamma = 1.0f - alpha - beta;

	//if all coordinates are none negative then the point is in the triangle
	return alpha >= 0 && beta >= 0 && gamma >= 0;
}

int FVCOMStructure::getContainingTriangle(point testPoint)
{
	//Get the closest node to start the search for the containing triangle
	int closestNode = getClosestNode(testPoint);

	//Search all triangles that are connected to the closest node
	for(int i = 0; i < nodeToTriangles[closestNode].size(); i++)
	{
		//return the triangle for which the point is inside
		if(pointInTriangle(testPoint, nodeToTriangles[closestNode][i]))
		{
			return nodeToTriangles[closestNode][i];
		}
	}

	//if the point is not inside any of those triangles search all the triangles
	for(int i = 0; i < triangles.size(); i++)
	{
		if(pointInTriangle(testPoint, i))
		{
			return i;
		}
	}

	return -1;
}

int FVCOMStructure::getClosestNode(point testPoint)
{
	//Checks distance between testPoint and every node, this is slow and will probably need to be improved
	float closestDistance = std::numeric_limits<float>::max();
	int node = -1;
	for(int i = 0; i < nodes.size(); i++)
	{
		if(distance(testPoint, nodes[i]) < closestDistance)
		{
			closestDistance = distance(testPoint, nodes[i]);
			node = i;
		}
	}

	return node;
}

float FVCOMStructure::distance(point p0, point p1)
{
	return std::sqrt( (p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)*(p0.y - p1.y) );
}


int FVCOMStructure::getChunkForNode(int node, int siglay, int time)
{
	//Chunk ids based on this ordering (x,y,sigma,time)

	int nodeX = nodes[node].x;
	int nodeY = nodes[node].y;

	//calculate the chunks for each individual dimension
	int xChunk = (nodeX / (maxX - minX)) * xDimChunks;
	int yChunk = (nodeX / (maxY - minY)) * yDimChunks;
	int siglayChunk = ((double)siglay / siglayDim) * siglayDimChunks;
	int timeChunk = ((double)time / times.size()) * timeDimChunks;

	return timeChunk +
		   (siglayChunk * timeDimChunks) +
		   (yChunk * timeDimChunks * siglayDimChunks) +
		   (xChunk * timeDimChunks * siglayDimChunks * yDimChunks);
}


int FVCOMStructure::getChunkForTriangle(int triangle, int siglay, int time)
{
	//Chunk ids based on this ordering (x,y,sigma,time)

	int triangleX = triangles[triangle].x;
	int triangleY = triangles[triangle].y;

	//calculate the chunks for each individual dimension
	int xChunk = (triangleX / (maxX - minX)) * xDimChunks;
	int yChunk = (triangleY / (maxY - minY)) * yDimChunks;
	int siglayChunk = ((double)siglay / siglayDim) * siglayDimChunks;
	int timeChunk = ((double)time / times.size()) * timeDimChunks;

	return timeChunk +
		   (siglayChunk * timeDimChunks) +
		   (yChunk * timeDimChunks * siglayDimChunks) +
		   (xChunk * timeDimChunks * siglayDimChunks * yDimChunks);
}