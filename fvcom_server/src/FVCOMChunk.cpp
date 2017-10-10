#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOMStructure.h"


#include <unordered_map>
#include <vector>

#include <netcdf>

FVCOMChunk::FVCOMChunk(const netCDF::NcFile& dataFile, std::vector<unsigned int> nodesToLoad,
											   std::vector<unsigned int> trianglesToLoad,
											   FVCOMStructure::ChunkInfo chunkInfo) :
	chunkInfo(chunkInfo)
{
	nodes.reserve(nodesToLoad.size());
	triangles.reserve(trianglesToLoad.size());

	std::vector<float> uLoad;
	std::vector<float> vLoad;
	std::vector<float> tempLoad;
	std::vector<float> saltLoad;
	

	netCDF::NcVar uVar = dataFile.getVar("u");
	netCDF::NcVar vVar = dataFile.getVar("v");
	netCDF::NcVar tempVar = dataFile.getVar("temp");
	netCDF::NcVar saltVar = dataFile.getVar("salinity");


	for(unsigned int i = 0; i < nodesToLoad.size(); i++)
	{		
		tempLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);
		saltLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);

		std::vector<size_t> start = {chunkInfo.timeStart, chunkInfo.siglayStart, nodesToLoad[i]};
		std::vector<size_t> count = {chunkInfo.timeSize, chunkInfo.siglaySize, 1};
		tempVar.getVar(start, count, tempLoad.data());
		saltVar.getVar(start, count, saltLoad.data());

		//Make new vector for all the NodeData objects
		nodes.insert(std::make_pair(nodesToLoad[i], std::vector<FVCOMChunk::NodeData>()));
		std::vector<FVCOMChunk::NodeData>& dataList = nodes[nodesToLoad[i]];
		dataList.resize(tempLoad.size());

		//populate vector of NodeData objects
		for(unsigned int j = 0; j < tempLoad.size(); j++)
		{
			FVCOMChunk::NodeData data;
			data.temp = tempLoad[j];
			data.salt = saltLoad[j];
			dataList[j] = data;
		}
	}

	for(unsigned int i = 0; i < trianglesToLoad.size(); i++)
	{		
		uLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);
		vLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);

		std::vector<size_t> start = {chunkInfo.timeStart, chunkInfo.siglayStart, trianglesToLoad[i]};
		std::vector<size_t> count = {chunkInfo.timeSize, chunkInfo.siglaySize, 1};
		uVar.getVar(start, count, uLoad.data());
		vVar.getVar(start, count, vLoad.data());


		//Make new vector for all the NodeData objects
		triangles.insert(std::make_pair(trianglesToLoad[i], std::vector<FVCOMChunk::TriangleData>()));
		std::vector<FVCOMChunk::TriangleData>& dataList = triangles[trianglesToLoad[i]];
		dataList.resize(uLoad.size());

		//populate vector of NodeData objects
		for(unsigned int j = 0 ; j < uLoad.size(); j++)
		{
			FVCOMChunk::TriangleData data;
			data.u = uLoad[j];
			data.v = vLoad[j];
			dataList[j] = data;
		}
	}
}

const FVCOMChunk::NodeData& FVCOMChunk::getNodeData(const unsigned int node, const unsigned int siglay, const unsigned int time)
{
	std::vector<FVCOMChunk::NodeData>& data = nodes[node];
	unsigned int index = (siglay - chunkInfo.siglayStart) + (time - chunkInfo.timeStart) * chunkInfo.siglaySize;

	return data[index];
}

const FVCOMChunk::TriangleData& FVCOMChunk::getTriangleData(const unsigned int triangle, const unsigned int siglay, const unsigned int time)
{
	const std::vector<FVCOMChunk::TriangleData>& data = triangles[triangle];
	unsigned int index = (siglay - chunkInfo.siglayStart) + (time - chunkInfo.timeStart) * chunkInfo.siglaySize;

	return data[index];
}