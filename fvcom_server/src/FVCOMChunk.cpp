#include "fvcom_server/FVCOMChunk.h"
#include "fvcom_server/FVCOMStructure.h"


#include <unordered_map>
#include <vector>

#include <netcdf>

FVCOMChunk::FVCOMChunk(const std::vector<FVCOMStructure::ModelFile> modelFiles, std::vector<unsigned int> nodesToLoad,
											   std::vector<unsigned int> trianglesToLoad,
											   FVCOMStructure::ChunkInfo chunkInfo) :
	chunkInfo(chunkInfo)
{

	unsigned int startModelFile = getFileIndexForTimeIndex(modelFiles, chunkInfo.timeStart);
	unsigned int endModelFile = getFileIndexForTimeIndex(modelFiles, chunkInfo.timeStart + chunkInfo.timeSize);

	nodes.reserve(nodesToLoad.size());
	triangles.reserve(trianglesToLoad.size());

	std::vector<float> uLoad;
	std::vector<float> vLoad;
	std::vector<float> tempLoad;
	std::vector<float> saltLoad;	

	//Initalize node data storage
	for(unsigned int i = 0; i < nodesToLoad.size(); i++)
	{
		tempLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);
		saltLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);

		nodes.insert(std::make_pair(nodesToLoad[i], std::vector<FVCOMChunk::NodeData>()));
		nodes[nodesToLoad[i]].resize(tempLoad.size());
	}

	//Initalize triange data storage
	for(unsigned int i = 0; i < trianglesToLoad.size(); i++)
	{
		uLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);
		vLoad.resize(chunkInfo.timeSize * chunkInfo.siglaySize);

		triangles.insert(std::make_pair(trianglesToLoad[i], std::vector<FVCOMChunk::TriangleData>()));
		triangles[trianglesToLoad[i]].resize(uLoad.size());
	}

	std::vector<netCDF::NcFile> dataFiles;

	for(unsigned int i = startModelFile; i <= endModelFile; i++)
	{
		dataFiles.push_back(netCDF::NcFile(modelFiles[i].filename, netCDF::NcFile::read));
	}


	for(unsigned int i = 0; i < nodesToLoad.size(); i++)
	{
		unsigned int timeIndex = chunkInfo.timeStart;
		unsigned int dataIndex = 0;

		for(unsigned int i = startModelFile; i <= endModelFile; i++)
		{
			netCDF::NcVar tempVar = dataFiles[i - startModelFile].getVar("temp");
			netCDF::NcVar saltVar = dataFiles[i - startModelFile].getVar("salinity");
			//calculate the size of the time dimension that needs to be loaded
			unsigned int timeCount = std::min(chunkInfo.timeSize, modelFiles[i].timeDim);

			//set start and count variables
			std::vector<size_t> start = {timeIndex, chunkInfo.siglayStart, nodesToLoad[i]};
			std::vector<size_t> count = {timeCount, chunkInfo.siglaySize, 1};

			//load data
			tempVar.getVar(start, count, tempLoad.data() + dataIndex);
			saltVar.getVar(start, count, saltLoad.data() + dataIndex);

			//Update time and data indicies
			timeIndex += timeCount;
			dataIndex += timeCount * chunkInfo.siglaySize;
		}


		std::vector<FVCOMChunk::NodeData>& dataList = nodes[nodesToLoad[i]];

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
		unsigned int timeIndex = chunkInfo.timeStart;
		unsigned int dataIndex = 0;

		for(unsigned int i = startModelFile; i <= endModelFile; i++)
		{
			netCDF::NcVar uVar = dataFiles[i - startModelFile].getVar("u");
			netCDF::NcVar vVar = dataFiles[i - startModelFile].getVar("v");

			//calculate the size of the time dimension that needs to be loaded
			unsigned int timeCount = std::min(chunkInfo.timeSize, modelFiles[i].timeDim);

			//set start and count variables
			std::vector<size_t> start = {timeIndex, chunkInfo.siglayStart, trianglesToLoad[i]};
			std::vector<size_t> count = {timeCount, chunkInfo.siglaySize, 1};

			uVar.getVar(start, count, uLoad.data() + dataIndex);
			vVar.getVar(start, count, vLoad.data() + dataIndex);

			//Update time and data indicies
			timeIndex += timeCount;
			dataIndex += timeCount * chunkInfo.siglaySize;
		}

		std::vector<FVCOMChunk::TriangleData>& dataList = triangles[trianglesToLoad[i]];

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

const unsigned int FVCOMChunk::getFileIndexForTimeIndex(const std::vector<FVCOMStructure::ModelFile> modelFiles, const unsigned int timeIndex) const
{
	for(int i = 0; i < modelFiles.size(); i++)
	{
		if(modelFiles[0].startTimeIndex >= timeIndex)
		{
			return i;
		}
	}

	return modelFiles.size();
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