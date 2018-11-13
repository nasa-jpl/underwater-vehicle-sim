#ifndef VEHICLE_POSE_H
#define VEHICLE_POSE_H


class VehiclePose
{
public:
    VehiclePose();
    VehiclePose(double x, double y, double z);
    ~VehiclePose() {}

    double getX() const;
    double getY() const;
    double getZ() const;

    void setX(double x);
    void setY(double y);
    void setZ(double z);

private:
    double x;
    double y;
    double z;
};

#endif