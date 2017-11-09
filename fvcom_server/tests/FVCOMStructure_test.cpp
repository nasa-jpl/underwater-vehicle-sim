#include "fvcom_server/FVCOMStructure.h"
#include <gtest/gtest.h>

const FVCOMStructure structure("test_data/box_plume_split", 10, 10, 10, 10);


TEST(FVCOMStructureTest, GetClosestTime) {
	int timeExact1 = structure.getClosestTime(0.0020833334); //Index 1
	int timeExact2 = structure.getClosestTime(0.05); //Index 24

	int timeNotExact1 = structure.getClosestTime(0.048957333); //Index 23
	int timeNotExact2 = structure.getClosestTime(0.048959333); //Index 24

	int timeSmallerThanZero = structure.getClosestTime(-0.1);
	int timeGreaterThanMax = structure.getClosestTime(1);


	ASSERT_EQ(1, timeExact1);
	ASSERT_EQ(24, timeExact2);

	ASSERT_EQ(23, timeNotExact1);
	ASSERT_EQ(24, timeNotExact2);

	ASSERT_EQ(0, timeSmallerThanZero);
	ASSERT_EQ(80, timeGreaterThanMax);
}


TEST(FCVOMStructureTest, GetClosestNodeSiglay) {
	FVCOMStructure::point p1;
	FVCOMStructure::point p2;
	FVCOMStructure::point p3;

	//Node 51
	//h = 300
	//siglay = 0
	p1.x = 70;
	p1.y = -100;
	p1.h = 0;

	//Node 51
	//h = 300
	//siglay = 69
	p2.x = 70;
	p2.y = -100;
	p2.h = -210.606051;

	//Node 51
	//h = 300
	//siglay = 70
	p3.x = 70;
	p3.y = -100;
	p3.h = -212.1212905;


	int surfaceExact = structure.getClosestNodeSiglay(p1);
	int depthExact = structure.getClosestNodeSiglay(p2);

	int depthNotExact = structure.getClosestNodeSiglay(p3);

	ASSERT_EQ(0, surfaceExact);
	ASSERT_EQ(69, depthExact);
	ASSERT_EQ(70, depthNotExact);
}


TEST(FCVOMStructureTest, GetClosestTriangleSiglay) {
	FVCOMStructure::point p1;
	FVCOMStructure::point p2;
	FVCOMStructure::point p3;

	//Node 81
	//h = 300
	//siglay = 0
	p1.x = -15.735039;
	p1.y = -17.163147;
	p1.h = 0;

	//Node 81
	//h = 300
	//siglay = 69
	p2.x = -15.735039;
	p2.y = -17.163147;
	p2.h = -210.606051;

	//Node 81
	//h = 300
	//siglay = 70
	p3.x = -15.735039;
	p3.y = -17.163147;
	p3.h = -212.1212905;


	int surfaceExact = structure.getClosestTriangleSiglay(p1);
	int depthExact = structure.getClosestTriangleSiglay(p2);

	int depthNotExact = structure.getClosestTriangleSiglay(p3);

	ASSERT_EQ(0, surfaceExact);
	ASSERT_EQ(69, depthExact);
	ASSERT_EQ(70, depthNotExact);
}

TEST(FVCOMStructureTest, PointInTriangle) {
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

TEST(FVCOMStructureTest, GetContainingTriangle) {
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


TEST(FVCOMStructureTest, GetClosestNode) {
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


TEST(FVCOMStructureTest, Distance) {
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


TEST(FVCOMStructureTest, GetChunkForNode) {

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

	FVCOMStructure::ChunkInfo chunk1 = structure.getChunkForNode(385, 0, 0);
	FVCOMStructure::ChunkInfo chunk2 = structure.getChunkForNode(385, 0, 80);
	FVCOMStructure::ChunkInfo chunk3 = structure.getChunkForNode(385, 54, 80);

	FVCOMStructure::ChunkInfo chunk4 = structure.getChunkForNode(0, 0, 0);
	FVCOMStructure::ChunkInfo chunk5 = structure.getChunkForNode(4, 0, 0);

	ASSERT_EQ(9990, chunk1.id);
	ASSERT_EQ(5, chunk1.xChunk);
	ASSERT_EQ(11, chunk1.yChunk);
	ASSERT_EQ(0, chunk1.siglayChunk);
	ASSERT_EQ(0, chunk1.timeChunk);

	ASSERT_EQ(9998, chunk2.id);
	ASSERT_EQ(5, chunk2.xChunk);
	ASSERT_EQ(11, chunk2.yChunk);
	ASSERT_EQ(0, chunk2.siglayChunk);
	ASSERT_EQ(8, chunk2.timeChunk);
	

	ASSERT_EQ(10043, chunk3.id);
	ASSERT_EQ(5, chunk3.xChunk);
	ASSERT_EQ(11, chunk3.yChunk);
	ASSERT_EQ(5, chunk3.siglayChunk);
	ASSERT_EQ(8, chunk3.timeChunk);

	ASSERT_EQ(0, chunk4.id);
	ASSERT_EQ(0, chunk4.xChunk);
	ASSERT_EQ(0, chunk4.yChunk);
	ASSERT_EQ(0, chunk4.siglayChunk);
	ASSERT_EQ(0, chunk4.timeChunk);

	ASSERT_EQ(34200, chunk5.id);
	ASSERT_EQ(19, chunk5.xChunk);
	ASSERT_EQ(0, chunk5.yChunk);
	ASSERT_EQ(0, chunk5.siglayChunk);
	ASSERT_EQ(0, chunk5.timeChunk);
}



TEST(FVCOMStructureTest, GetChunkForTriangle) {
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

	FVCOMStructure::ChunkInfo chunk1 = structure.getChunkForTriangle(1, 0, 0);
	FVCOMStructure::ChunkInfo chunk2 = structure.getChunkForTriangle(1, 0, 80);
	FVCOMStructure::ChunkInfo chunk3 = structure.getChunkForTriangle(1, 54, 80);


	ASSERT_EQ(8190, chunk1.id);
	ASSERT_EQ(4, chunk1.xChunk);
	ASSERT_EQ(11, chunk1.yChunk);
	ASSERT_EQ(0, chunk1.siglayChunk);
	ASSERT_EQ(0, chunk1.timeChunk);

	ASSERT_EQ(8198, chunk2.id);
	ASSERT_EQ(4, chunk2.xChunk);
	ASSERT_EQ(11, chunk2.yChunk);
	ASSERT_EQ(0, chunk2.siglayChunk);
	ASSERT_EQ(8, chunk2.timeChunk);

	ASSERT_EQ(8243, chunk3.id);
	ASSERT_EQ(4, chunk3.xChunk);
	ASSERT_EQ(11, chunk3.yChunk);
	ASSERT_EQ(5, chunk3.siglayChunk);
	ASSERT_EQ(8, chunk3.timeChunk);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}