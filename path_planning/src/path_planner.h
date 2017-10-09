#ifndef __PATH_PLANNER__
#define __PATH_PLANNER__

#include <vector>
#include <memory>

struct Path {
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  Path() {}
  Path(Path* p) {
    x_vals = p->x_vals;
    y_vals = p->y_vals;
  }
};

class PathPlanner {
public:
  PathPlanner() {}
  ~PathPlanner() {}

  std::unique_ptr<Path> getStraightLinePath(
    double car_x,
    double car_y,
    double car_yaw,
    double dist_inc);

  // std::unique_ptr<Path> getCurvedPath();
};

#endif
