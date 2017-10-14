#ifndef VECHICLE_H
#define VECHICLE_H

#include "constants.h"

struct EgoVehicle {
  double mX;
  double mY;
  double mV;
  double mS;
  double mD;
  double mYaw;

  void resetState(
    const double x,
    const double y,
    const double v,
    const double s,
    const double d,
    const double yaw) {
      mX = x;
      mY = y;
      mV = v;
      mS = s;
      mD = d;
      mYaw = yaw;
  }
};


struct Vehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

#endif // VECHICLE_H
