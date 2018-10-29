#ifndef VEHICLE_INFO_H
#define VEHICLE_INFO_H

#include "underwater_vehicle_msgs/GetVehicleInfo.h"

class VehicleInfo
{

public:
    VehicleInfo(underwater_vehicle_msgs::GetVehicleInfo info);
    ~VehicleInfo() {}

    const std::string getName();
    const std::string getPropModuleName();
    const std::string getPropModuleType();

    const std::vector<std::string> getModuleNames();
    const std::vector<std::string> getModuleTypes();

    const std::vector<std::string> getModuleNamesOfType(const std::string type);

private:
    std::string name;
    std::string propModuleName;
    std::string propModuleType;
    std::vector<std::string> moduleNames;
    std::vector<std::string> moduleTypes;
};


#endif