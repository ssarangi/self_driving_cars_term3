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
  PathPlanner(
      const std::vector<double>& map_waypoints_x,
      const std::vector<double>& map_waypoints_y,
      const std::vector<double>& map_waypoints_s) {
    m_currentLane = 1;
    m_refVel = 0; // mph
    m_mapWaypoints_x = map_waypoints_x;
    m_mapWaypoints_y = map_waypoints_y;
    m_mapWaypoints_s = map_waypoints_s;
  }
  ~PathPlanner() {}

  std::unique_ptr<Path> getStraightLinePath(
    double car_x,
    double car_y,
    double car_yaw,
    double dist_inc);

  std::unique_ptr<Path> generateTrajectory(
      double car_x,
      double car_y,
      double car_s,
      double car_d,
      double car_yaw,
      double car_speed,
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      const double end_path_s,
      const double end_path_d,
      const std::vector<std::vector<double>>& sensor_fusion);

private:
  int m_currentLane;
  double m_refVel;
  std::vector<double> m_mapWaypoints_x;
  std::vector<double> m_mapWaypoints_y;
  std::vector<double> m_mapWaypoints_s;
};

#endif
