#ifndef VECHICLE_H
#define VECHICLE_H

#include "constants.h"

struct VehicleState {
    double x;
    double y;
    double v;
    double a;
    double s;
    double d;
    double yaw;
};

class Vehicle {
public:
    Vehicle() {}
    Vehicle(int id, double x, double y, double v, double a, double s, double d, double yaw);
    ~Vehicle() {}

    LANE getLane();
    double getX() { return m_vehicleState.x; }
    double getY() { return m_vehicleState.y; }
    double getV() { return m_vehicleState.v; }
    double getA() { return m_vehicleState.a; }
    double getS() { return m_vehicleState.s; }
    double getD() { return m_vehicleState.d; }
    double getYaw() { return m_vehicleState.yaw; }

private:
    VehicleState m_vehicleState;
};

#endif // VECHICLE_H
