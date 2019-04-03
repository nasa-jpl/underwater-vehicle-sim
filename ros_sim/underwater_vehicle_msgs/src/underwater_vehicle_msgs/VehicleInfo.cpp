#include "underwater_vehicle_msgs/VehicleInfo.h"

VehicleInfo::VehicleInfo() {}

VehicleInfo::VehicleInfo(underwater_vehicle_msgs::GetVehicleInfo info) :
    name(info.request.name),
    propModuleName(info.response.propModuleName),
    propModuleType(info.response.propModuleType),
    moduleNames(info.response.moduleNames),
    moduleTypes(info.response.moduleTypes),
    startX(info.response.startX),
    startY(info.response.startY),
    startZ(info.response.startZ)
{}

const std::string VehicleInfo::getName()
{
    return name;
}

const std::string VehicleInfo::getPropModuleName()
{
    return propModuleName;
}

const std::string VehicleInfo::getPropModuleType()
{
    return propModuleType;
}

const std::vector<std::string> VehicleInfo::getModuleNames()
{
    return moduleNames;
}

const std::vector<std::string> VehicleInfo::getModuleTypes()
{
    return moduleTypes;
}

const double VehicleInfo::getStartX()
{
    return startX;
}

const double VehicleInfo::getStartY()
{
    return startY;
}

const double VehicleInfo::getStartZ()
{
    return startZ;
}

const std::vector<std::string> VehicleInfo::getModuleNamesOfType(const std::string type)
{
    std::vector<std::string> returnNames;
    for(unsigned int i = 0; i < moduleTypes.size(); i++)
    {
        if(moduleTypes[i] == type)
        {
            returnNames.push_back(moduleNames[i]);
        }
    }
    return returnNames;
}