#include <math.h>
#include <utility>
#include <memory>
#include <cstring>
#include <assert.h>

#include "spline.h"
#include "path_planner.h"
#include "utils.h"

using namespace std;

typedef const vector<double>* vector_ptr;

void PathPlanner::initializeEgoVehicle(
    const double x,
    const double y,
    const double v,
    const double s,
    const double d,
    const double yaw) {
  m_pEgoVehicle->resetState(x, y, v, s, d, yaw);
}

void PathPlanner::initializeTraffic(
  const std::vector<std::vector<double>>& sensor_fusion) {
  // First find out
  int total_vehicles = sensor_fusion.size();
  for (int i = 0; i < total_vehicles; ++i) {
    int id = sensor_fusion[i][0];
    const vector_ptr pData = static_cast<vector_ptr>(&sensor_fusion[i]);
    if (m_IdToVehicle.find(id) == m_IdToVehicle.end()) {
      m_IdToVehicle[id] = new Vehicle();
    }

    assert(pData->size() * sizeof(double) == sizeof(Vehicle));
    memcpy(m_IdToVehicle[id], pData, pData->size() * sizeof(double));
  }
}

bool PathPlanner::checkClosenessToOtherCarsAndChangeLanes(
        const vector<vector<double>>& sensor_fusion,
        const int previous_iteration_points_left,
        const double car_s) {
  // Check CLoseness with other cars.
  bool too_close = false;

  for (int i = 0; i < sensor_fusion.size(); ++i) {
    float d = sensor_fusion[i][6];

    if (d < (2+4*m_currentLane+2) && d > (2+4*m_currentLane-2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)previous_iteration_points_left * 0.02 * check_speed); // If using previous points can project s value output
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
        too_close = true;
        if (m_currentLane > 0)
          m_currentLane -= 1;
        else if (m_currentLane < 3)
          m_currentLane += 1;
      }
    }
  }

  return too_close;
}

void PathPlanner::reduceOrIncreaseReferenceVelocity(bool too_close) {
  if (too_close)
    m_refVel -= 0.224 ;
  else if (m_refVel < 49.5)
    m_refVel += 0.224;
}

unique_ptr<Path> PathPlanner::createTrajectoryPoints(
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y) {

  int prev_path_size = previous_path_x.size();

  // Trajectory Generation.
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = m_pEgoVehicle->mX;
  double ref_y = m_pEgoVehicle->mY;
  double ref_yaw = deg2rad(m_pEgoVehicle->mYaw);

  if (prev_path_size < 2) {
    double prev_car_x = m_pEgoVehicle->mX - cos(m_pEgoVehicle->mYaw);
    double prev_car_y = m_pEgoVehicle->mY - sin(m_pEgoVehicle->mYaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(m_pEgoVehicle->mX);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(m_pEgoVehicle->mY);
  } else {
    ref_x = previous_path_x[prev_path_size - 1];
    ref_y = previous_path_y[prev_path_size - 1];

    double ref_x_prev = previous_path_x[prev_path_size - 2];
    double ref_y_prev = previous_path_y[prev_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(m_pEgoVehicle->mS + 30, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp1 = getXY(m_pEgoVehicle->mS + 60, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp2 = getXY(m_pEgoVehicle->mS + 90, (2 + 4 * m_currentLane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);

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

unique_ptr<Path> PathPlanner::generateTrajectory(
    const double car_x,
    const double car_y,
    const double car_s,
    const double car_d,
    const double car_yaw,
    const double car_speed,
    const double end_path_s,
    const double end_path_d,
    const vector<double>& previous_path_x,
    const vector<double>& previous_path_y,
    const vector<vector<double>>& sensor_fusion) {

  int prev_path_size = previous_path_x.size();

  double ego_car_s = prev_path_size > 0 ? end_path_s : car_s;

  // Initialization of Ego Vehicle
  initializeEgoVehicle(car_x, car_y, car_speed, ego_car_s, car_d, car_yaw);

  // Initialize the traffic. Create Vehicle objects for the id's not yet seen and then find out the closest
  // vehicles and get them in a vector.
  initializeTraffic(sensor_fusion);

  // Check CLoseness with other cars.
  bool too_close = checkClosenessToOtherCarsAndChangeLanes(sensor_fusion, prev_path_size, car_s);

  reduceOrIncreaseReferenceVelocity(too_close);

  auto p = createTrajectoryPoints(previous_path_x, previous_path_y);
  return p;
}
