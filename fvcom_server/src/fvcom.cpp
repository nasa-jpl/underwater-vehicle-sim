#include "fvcom_server/fvcom.h"

#include <netcdf>
#include <memory>

FVCOM::FVCOM(std::string filename) :
	dataFile(netCDF::NcFile(filename, netCDF::NcFile::read)),
	nodeDim(dataFile.getDim("node").getSize()),
	neleDim(dataFile.getDim("nele").getSize())
{
	//Load all variables for the structure of the model
	netCDF::NcVar xVar = dataFile.getVar("x");
	netCDF::NcVar yVar = dataFile.getVar("y");
	netCDF::NcVar xcVar = dataFile.getVar("xc");
	netCDF::NcVar ycVar = dataFile.getVar("yc");
	netCDF::NcVar nvVar = dataFile.getVar("nv");

	x.resize(nodeDim);
	y.resize(nodeDim);
	xc.resize(neleDim);
	yc.resize(neleDim);

	//resize for multidimensional array
	nv.resize(3);
	for(int i = 0; i < 3; i++)
	{
		nv[i].resize(neleDim);
	}


	//Assign all arrays for the structure variables
	xVar.getVar(x.data());
	yVar.getVar(y.data());
	xcVar.getVar(xc.data());
	ycVar.getVar(yc.data());

	//load nvVar into a multidimensional vector
	for(unsigned int i = 0; i < 3; i++)
	{
		std::vector<size_t> start = {i, 0};
		std::vector<size_t> count = {1, neleDim};
		nvVar.getVar(start, count, nv[i].data());
	}
}