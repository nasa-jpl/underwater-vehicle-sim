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

private:
	const netCDF::NcFile dataFile;
	const unsigned long nodeDim;
	const unsigned long neleDim;

	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> xc;
	std::vector<float> yc;
	std::vector<std::vector<int>> nv;

};

#endif