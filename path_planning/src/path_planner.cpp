#include <math.h>
#include <utility>
#include <memory>

#include "path_planner.h"
#include "utils.h"

using namespace std;

unique_ptr<Path> PathPlanner::getStraightLinePath(
    double car_x,
    double car_y,
    double car_yaw,
    double dist_inc) {
  Path *pStraightPath = new Path();

  for (int i = 0; i < 1000; ++i) {
    pStraightPath->x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    pStraightPath->y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
  }

  auto p = std::make_unique<Path>(move(pStraightPath));
  return p;
}
