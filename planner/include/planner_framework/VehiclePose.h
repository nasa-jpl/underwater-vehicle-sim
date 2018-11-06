#ifndef VEHICLE_POSE_H
#define VEHICLE_POSE_H


class VehiclePose
{
public:
    VehiclePose();
    VehiclePose(double x, double y, double z);
    ~VehiclePose() {}

    double getX();
    double getY();
    double getZ();

private:
    double x;
    double y;
    double z;
};

#endif