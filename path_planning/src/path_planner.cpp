#include <math.h>
#include <utility>
#include <memory>

#include "spline.h"
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

unique_ptr<Path> PathPlanner::generateTrajectory(
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    double car_speed,
    const vector<double>& previous_path_x,
    const vector<double>& previous_path_y,
    const double end_path_s,
    const double end_path_d,
    const vector<vector<double>>& sensor_fusion) {
  int prev_size = previous_path_x.size();

  if (prev_size > 0) {
    car_s = end_path_s;
  }


  // Check CLoseness with other cars.
  bool too_close = false;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    float d = sensor_fusion[i][6];

    if (d < (2+4*m_currentLane+2) && d > (2+4*m_currentLane-2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)prev_size * 0.02 * check_speed); // If using previous points can project s value output
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
        too_close = true;
        if (m_currentLane > 0)
          m_currentLane -= 1;
        else if (m_currentLane < 3)
          m_currentLane += 1;
      }
    }
  }

  if (too_close)
    m_refVel -= 0.224;
  else if (m_refVel < 49.5)
    m_refVel += 0.224;

  // Trajectory Generation.
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); ++i) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // Create a spline
  tk::spline s;

  s.set_points(ptsx, ptsy);

  Path *pTrajectory = new Path();

  for (int i = 0; i < previous_path_x.size(); ++i) {
    pTrajectory->x_vals.push_back(previous_path_x[i]);
    pTrajectory->y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

  double x_add_on= 0;
  double N = (target_dist / (0.02 * m_refVel/2.24));
  for (int i = 0; i <= 50 - previous_path_x.size(); ++i) {
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

     // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    pTrajectory->x_vals.push_back(x_point);
    pTrajectory->y_vals.push_back(y_point);
  }

  auto p = std::make_unique<Path>(move(pTrajectory));
  return p;
}
