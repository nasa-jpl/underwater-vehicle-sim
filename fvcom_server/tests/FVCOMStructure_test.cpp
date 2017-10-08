#include "fvcom_server/FVCOMStructure.h"
#include <gtest/gtest.h>


TEST(FCVOMStructureTest, PointInTriangle) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 500, 500, 10, 10);

	FVCOMStructure::point pIn;
	FVCOMStructure::point pOut;
	FVCOMStructure::point pEdge;


	//Triangle 1
	////Nodes: 385, 329,342
	//////X,Y 385: -48.038475, 10
	//////X,Y 329: -56.69873, 5
	//////X,Y 342: -56.69873, 15.0
	pEdge.x = -48.038475;
	pEdge.y = 10;
	pOut.x = 0;
	pOut.y = 0;
	pIn.x = -50;
	pIn.y = 10;



	bool pInResults = structure.pointInTriangle(pIn, 1);
	bool pOutResults = structure.pointInTriangle(pOut, 1);
	bool pEdgeResults = structure.pointInTriangle(pEdge, 1);


	ASSERT_TRUE(pInResults);
	ASSERT_FALSE(pOutResults);
	ASSERT_TRUE(pEdgeResults);
}

TEST(FCVOMStructureTest, GetContainingTriangle) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 500, 500, 10, 10);

	FVCOMStructure::point pIn;
	FVCOMStructure::point pOut;
	FVCOMStructure::point pEdge;

	//Triangle 1
	////Nodes: 385, 329,342
	//////X,Y 385: -48.038475, 10
	//////X,Y 329: -56.69873, 5
	//////X,Y 342: -56.69873, 15.0
	pEdge.x = -48.038475;
	pEdge.y = 10;
	pOut.x = 0;
	pOut.y = 0;
	pIn.x = -50;
	pIn.y = 10;


	int pInResults = structure.getContainingTriangle(pIn);
	int pOutResults = structure.getContainingTriangle(pOut);
	int pEdgeResults = structure.getContainingTriangle(pEdge);

	ASSERT_EQ(1, pInResults);
	ASSERT_EQ(1, pEdgeResults);
	ASSERT_NE(1, pOutResults);
}


TEST(FCVOMStructureTest, GetClosestNode) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 500, 500, 10, 10);

	FVCOMStructure::point p1;
	FVCOMStructure::point p2;
	FVCOMStructure::point p3;

	//Triangle 1
	////Nodes: 385, 329,342
	//////X,Y 385: -48.038475, 10
	//////X,Y 329: -56.69873, 5
	//////X,Y 342: -56.69873, 15.0
	p1.x = -48.038475;
	p1.y = 10;
	p2.x = -56.69873;
	p2.y = 10;
	p3.x = -57;
	p3.y = 6;

	int p1Results = structure.getClosestNode(p1);
	int p2Results = structure.getClosestNode(p2);
	int p3Results = structure.getClosestNode(p3);


	ASSERT_EQ(385, p1Results);
	ASSERT_EQ(329, p2Results);
	ASSERT_EQ(329, p3Results);
}


TEST(FCVOMStructureTest, Distance) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 500, 500, 10, 10);

	FVCOMStructure::point p1;
	FVCOMStructure::point p2;
	FVCOMStructure::point p3;

	p1.x = 0;
	p1.y = 0;
	p2.x = 5;
	p2.y = 0;
	p3.x = 0;
	p3.y = 7;

	ASSERT_FLOAT_EQ(5.0, structure.distance(p1, p2));
	ASSERT_FLOAT_EQ(5.0, structure.distance(p2, p1));

	ASSERT_FLOAT_EQ(7.0, structure.distance(p1, p3));
	ASSERT_FLOAT_EQ(7.0, structure.distance(p3, p1));

	ASSERT_FLOAT_EQ(8.60232526704, structure.distance(p3, p2));
	ASSERT_FLOAT_EQ(8.60232526704, structure.distance(p2, p3));
}


TEST(FCVOMStructureTest, GetChunkForNode) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 10, 10, 10, 10);


	//model extent
	////x: -100 , 100
	////y: -100 , 100
	////siglay: 99
	////time: 81

	////Nodes: 385
	//////X,Y 385: -48.038475, 10

	//x Chunk: 5
	//y Chunk: 11
	//siglay: 0
	//time: 0
	//CHUNK: 9990

	//siglay: 0
	//time: 80
	//CHUNK: 9998

	//siglay: 54
	//time: 80
	//CHUNK: 10043

	ASSERT_EQ(9990, structure.getChunkForNode(385, 0, 0));
	ASSERT_EQ(9998, structure.getChunkForNode(385, 0, 80));
	ASSERT_EQ(10043, structure.getChunkForNode(385, 54, 80));
}



TEST(FCVOMStructureTest, GetChunkForTriangle) {
    netCDF::NcFile dataFile("test_data/box_plume_0001.nc", netCDF::NcFile::read);
	FVCOMStructure structure(dataFile, 10, 10, 10, 10);

	//model extent
	////x: -100 , 100
	////y: -100 , 100
	////siglay: 99
	////time: 81

	////Triangle: 1
	//////X,Y 1: -53.81198, 10

	//x Chunk: 4
	//y Chunk: 11
	//siglay: 0
	//time: 0
	//CHUNK: 8190

	//siglay: 0
	//time: 80
	//CHUNK: 8198

	//siglay: 54
	//time: 80
	//CHUNK: 8243

	ASSERT_EQ(8190, structure.getChunkForTriangle(1, 0, 0));
	ASSERT_EQ(8198, structure.getChunkForTriangle(1, 0, 80));
	ASSERT_EQ(8243, structure.getChunkForTriangle(1, 54, 80));

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}