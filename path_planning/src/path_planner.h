#ifndef __PATH_PLANNER__
#define __PATH_PLANNER__

#include <vector>
#include <memory>
#include <unordered_map>

#include "vehicle.h"

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
    m_pEgoVehicle = new EgoVehicle();
    counter = 0;
  }
  ~PathPlanner() {
    // Free up all the objects we allocated
  }

  std::unique_ptr<Path> generateTrajectory(
      const double car_x,
      const double car_y,
      const double car_s,
      const double car_d,
      const double car_yaw,
      const double car_speed,
      const double end_path_s,
      const double end_path_d,
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      const std::vector<std::vector<double>>& sensor_fusion);

private:
  void initializeEgoVehicle(
      const double x,
      const double y,
      const double v,
      const double s,
      const double d,
      const double yaw);

  void initializeTraffic(const std::vector<std::vector<double>>& sensor_fusion);

  bool checkClosenessToOtherCarsAndChangeLanes(
    const std::vector<std::vector<double>>& sensor_fusion,
    const int previous_iteration_points_left,
    const double car_s);

  void reduceOrIncreaseReferenceVelocity(bool too_close);

  void createPointsForSpline(
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      std::vector<double>& ptsx,
      std::vector<double>& ptsy);

  std::unique_ptr<Path> createTrajectoryPoints(
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y);


private:
  int counter;
  EgoVehicle *m_pEgoVehicle;
  int m_currentLane;
  double m_refVel;
  std::vector<double> m_mapWaypoints_x;
  std::vector<double> m_mapWaypoints_y;
  std::vector<double> m_mapWaypoints_s;
  std::unordered_map<int, Vehicle*> m_IdToVehicle;
  std::vector<const Vehicle*> m_currentIterationTraffic;
};

#endif
