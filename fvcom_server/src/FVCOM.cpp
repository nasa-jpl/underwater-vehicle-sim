#include "fvcom_server/FVCOM.h"

FVCOM::FVCOM(std::string filename) :
	dataFile(netCDF::NcFile(filename, netCDF::NcFile::read)),
	structure(FVCOMStructure(dataFile, 500, 500, 10, 10))
{
	
}

FVCOM::FVCOM(std::string filename, int xChunkSize, int yChunkSize, int siglayChunkSize, int timeChunkSize) :
	dataFile(netCDF::NcFile(filename, netCDF::NcFile::read)),
	structure(FVCOMStructure(dataFile, xChunkSize, yChunkSize, siglayChunkSize, timeChunkSize))
{
	
}