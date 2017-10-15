#include <math.h>
#include <utility>
#include <memory>
#include <cstring>
#include <assert.h>
#include <iostream>
#include <algorithm>

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
  m_LaneIdToVehicles.clear();

  // First find out
  int total_vehicles = sensor_fusion.size();
  for (int i = 0; i < total_vehicles; ++i) {
    int id = sensor_fusion[i][0];
    if (m_IdToVehicle.find(id) == m_IdToVehicle.end()) {
      m_IdToVehicle[id] = new Vehicle();
    }

    assert(sensor_fusion[0].size() * sizeof(double) == sizeof(Vehicle));
    // Copy the data from Sensor fusion to the vehicles
    m_IdToVehicle[id]->setData(sensor_fusion[i]);

    // Arrange the vehicles with the lanes
    float d = m_IdToVehicle[id]->d;
    int lane_id = getLaneId(d);
    if (lane_id >= 0 && lane_id <= 2) {
      m_LaneIdToVehicles[lane_id].push_back(m_IdToVehicle[id]);
    }
  }
}

bool PathPlanner::checkClosenessToOtherCarsAndChangeLanes(
        const vector<vector<double>>& sensor_fusion,
        const int previous_iteration_points_left,
        const double car_s) {
  // Check closeness with other cars.
  bool too_close = false;

  // Check only the cars in the current lane
  vector<Vehicle*> vehicles_in_ego_car_lane = m_LaneIdToVehicles[m_currentLane];
  if (vehicles_in_ego_car_lane.size() > 0)
    cout << "Found " << vehicles_in_ego_car_lane.size() << " cars in lane: " << m_currentLane << endl;
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    float d = sensor_fusion[i][6];
    cout << "D: " << d << endl;
  }

   for (Vehicle* vehicle : vehicles_in_ego_car_lane) {
    double vx = vehicle->vx;
    double vy = vehicle->vy;
    double check_car_s = vehicle->s;
    double check_speed = sqrt(vx * vx + vy * vy);

    check_car_s += ((double)previous_iteration_points_left * 0.02 * check_speed); // If using previous points can project s value output
    if ((check_car_s > m_pEgoVehicle->mS) && ((check_car_s - m_pEgoVehicle->mS) < SAFE_DISTANCE_TO_MAINTAIN)) {
      too_close = true;
      if (m_currentLane > 0) {
        m_currentLane -= 1;
      } else if (m_currentLane < 2) {
        m_currentLane += 1;
      }
      cout << "Changed lane to " << m_currentLane << endl;
    }
  }
  return too_close;
}

// Change the reference velocity for any vehicle.
double PathPlanner::reduceOrIncreaseReferenceVelocity(
    const bool too_close,
    const double oldRefVel) {
  double newRefVel = oldRefVel;

  if (too_close) {
    newRefVel -= 0.224;
    newRefVel = max(newRefVel, 0.0);
  } else if (oldRefVel < 49.5) {
    newRefVel += 0.224;
    newRefVel = min(newRefVel, MAX_SPEED);
  }

  return newRefVel;
}

Path* PathPlanner::createPointsForSpline(
    const EgoVehicle *pEgoVehicle,
    const int current_lane,
    const std::vector<double> &previous_path_x,
    const std::vector<double> &previous_path_y,
    double &ref_x,
    double &ref_y,
    double &ref_yaw) const {

  Path *pPointsOnSpline = new Path();
  int prev_path_size = previous_path_x.size();

  if (prev_path_size < 2) {
    double prev_car_x = pEgoVehicle->mX - cos(pEgoVehicle->mYaw);
    double prev_car_y = pEgoVehicle->mY - sin(pEgoVehicle->mYaw);

    pPointsOnSpline->x_vals.push_back(prev_car_x);
    pPointsOnSpline->x_vals.push_back(pEgoVehicle->mX);

    pPointsOnSpline->y_vals.push_back(prev_car_y);
    pPointsOnSpline->y_vals.push_back(pEgoVehicle->mY);
  } else {
    ref_x = previous_path_x[prev_path_size - 1];
    ref_y = previous_path_y[prev_path_size - 1];

    double ref_x_prev = previous_path_x[prev_path_size - 2];
    double ref_y_prev = previous_path_y[prev_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    pPointsOnSpline->x_vals.push_back(ref_x_prev);
    pPointsOnSpline->x_vals.push_back(ref_x);

    pPointsOnSpline->y_vals.push_back(ref_y_prev);
    pPointsOnSpline->y_vals.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(pEgoVehicle->mS + 30, (2 + 4 * current_lane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp1 = getXY(pEgoVehicle->mS + 60, (2 + 4 * current_lane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);
  vector<double> next_wp2 = getXY(pEgoVehicle->mS + 90, (2 + 4 * current_lane), m_mapWaypoints_s, m_mapWaypoints_x, m_mapWaypoints_y);

  pPointsOnSpline->x_vals.push_back(next_wp0[0]);
  pPointsOnSpline->x_vals.push_back(next_wp1[0]);
  pPointsOnSpline->x_vals.push_back(next_wp2[0]);

  pPointsOnSpline->y_vals.push_back(next_wp0[1]);
  pPointsOnSpline->y_vals.push_back(next_wp1[1]);
  pPointsOnSpline->y_vals.push_back(next_wp2[1]);

  for (int i = 0; i < pPointsOnSpline->x_vals.size(); ++i) {
    double shift_x = pPointsOnSpline->x_vals[i] - ref_x;
    double shift_y = pPointsOnSpline->y_vals[i] - ref_y;

    pPointsOnSpline->x_vals[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    pPointsOnSpline->y_vals[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  return pPointsOnSpline;
}

unique_ptr<Path> PathPlanner::interpolatePointsOnSpline(
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y,
    const Path *pPointsOnSpline,
    const double ref_x,
    const double ref_y,
    const double ref_yaw) const {

  // Create a spline
  tk::spline spline;

  spline.set_points(pPointsOnSpline->x_vals, pPointsOnSpline->y_vals);

  Path *pTrajectory = new Path();

  for (int i = 0; i < previous_path_x.size(); ++i) {
    pTrajectory->x_vals.push_back(previous_path_x[i]);
    pTrajectory->y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

  double x_add_on= 0;
  double N = (target_dist / (0.02 * m_refVel/2.24));
  for (int i = 0; i <= 50 - previous_path_x.size(); ++i) {
    double x_point = x_add_on + (target_x) / N;
    double y_point = spline(x_point);

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

unique_ptr<Path> PathPlanner::createTrajectoryPoints(
    const EgoVehicle *pEgoVehicle,
    const int current_lane,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y) const {

  // Trajectory Generation.
  // These values are changed in these functions.
  double ref_yaw = deg2rad(pEgoVehicle->mYaw);
  double ref_x = pEgoVehicle->mX;
  double ref_y = pEgoVehicle->mY;

  Path *pPointsForSpline = createPointsForSpline(
        pEgoVehicle,
        current_lane,
        previous_path_x,
        previous_path_y,
        ref_x,
        ref_y,
        ref_yaw);

  auto p = interpolatePointsOnSpline(
        previous_path_x,
        previous_path_y,
        pPointsForSpline,
        ref_x,
        ref_y,
        ref_yaw);

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
  double ego_car_d = prev_path_size > 0 ? end_path_d : car_d;

  // Initialization of Ego Vehicle
  initializeEgoVehicle(car_x, car_y, car_speed, ego_car_s, ego_car_d, car_yaw);

  // Initialize the traffic. Create Vehicle objects for the id's not yet seen and then find out the closest
  // vehicles and get them in a vector.
  initializeTraffic(sensor_fusion);

  // Check CLoseness with other cars.
  bool too_close = checkClosenessToOtherCarsAndChangeLanes(sensor_fusion, prev_path_size, car_s);

  // Change the reference velocity based on whether there are cars or not.
  m_refVel = reduceOrIncreaseReferenceVelocity(too_close, m_refVel);

  auto p = createTrajectoryPoints(
        m_pEgoVehicle,
        m_currentLane,
        previous_path_x,
        previous_path_y);
  return p;
}
