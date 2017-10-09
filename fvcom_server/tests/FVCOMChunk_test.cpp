#include "fvcom_server/FVCOMStructure.h"
#include "fvcom_server/FVCOMChunk.h"
#include <gtest/gtest.h>


const netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
const FVCOMStructure structure(dataFile, 500, 500, 10, 10);

TEST(FCVOMChunkTest, GetNodeData) {
	FVCOMStructure::ChunkInfo chunkInfo = structure.getChunkForNode(1,0,0);
	const std::vector<unsigned int>& nodes = structure.getNodesInChunk(chunkInfo);
	const std::vector<unsigned int>& triangles = structure.getTrianglesInChunk(chunkInfo);

	FVCOMChunk chunk(dataFile, nodes, triangles, chunkInfo);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}