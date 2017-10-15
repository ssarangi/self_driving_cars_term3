#ifndef VECHICLE_H
#define VECHICLE_H

#include <vector>
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

  void setData(const std::vector<double>& data) {
    id = data[0];
    x = data[1];
    y = data[2];
    vx = data[3];
    vy = data[4];
    s = data[5];
    d = data[6];
  }
};

#endif // VECHICLE_H
