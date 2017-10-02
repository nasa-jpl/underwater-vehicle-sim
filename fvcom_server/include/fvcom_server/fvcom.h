#ifndef FVCOM_H
#define FVCOM_H

#include <list>
#include <unordered_map>
#include <cstddef>
#include <stdexcept>
#include <memory>

#include <netcdf>

class FVCOM
{
public:
	FVCOM(std::string filename);

private:
	struct point
	{
		float x;
		float y;
		float h;
	};

	bool pointInTriangle(float px, float py, int triangle);
	int getContainingTriangle(float x, float y);
	int getClosestNode(float x, float y);
	float distance(float p0X, float p0Y, point p1);


private:
	const netCDF::NcFile dataFile;

	std::vector<point> nodes;
	std::vector<point> triangles;
	std::vector<float> time;

	std::vector<std::vector<int>> triangleToNodes;

	std::vector<std::vector<int>> nodeToTriangles;
};

#endif