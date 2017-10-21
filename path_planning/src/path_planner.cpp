#include <math.h>
#include <utility>
#include <memory>
#include <cstring>
#include <assert.h>
#include <iostream>
#include <algorithm>
#include <random>
#include <limits>
#include <thread>
#include <future>

#include "spline.h"
#include "path_planner.h"
#include "cost_function.h"
#include "utils.h"

using namespace std;

typedef const vector<double>* vector_ptr;

double PathPlanner::computeCostForTrajectory(
    EgoVehicleNewState *pNewState,
    const vector<double>& previous_path_x,
    const vector<double>& previous_path_y) {
  unique_ptr<Path> pPath = createTrajectoryPoints(
        m_pEgoVehicle,
        pNewState->new_lane,
        previous_path_x,
        previous_path_y);

  double current_cost = compute_cost(pPath.get(),
               m_pEgoVehicle,
               m_currentLane,
               pNewState->new_lane,
               pNewState->refVel,
               m_LaneIdToVehicles,
               m_mapWaypoints_x,
               m_mapWaypoints_y);

  return current_cost;
}

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

// Check if the ego vehicle is close to other cars in the same lane.
vector<EgoVehicleNewState*> PathPlanner::checkClosenessToOtherCarsAndChangeLanes(
        const int previous_iteration_points_left) {

  bool too_close = false;

  // Check only the cars in the current lane
  vector<Vehicle*> vehicles_in_ego_car_lane = m_LaneIdToVehicles[m_currentLane];

  vector<EgoVehicleNewState*> ego_vehicle_states;
  for (Vehicle* vehicle : vehicles_in_ego_car_lane) {
    double vx = vehicle->vx;
    double vy = vehicle->vy;
    double check_car_s = vehicle->s;
    double check_speed = sqrt(vx * vx + vy * vy);

    check_car_s += ((double)previous_iteration_points_left * 0.02 * check_speed); // If using previous points can project s value output
    if ((check_car_s > m_pEgoVehicle->mS) && ((check_car_s - m_pEgoVehicle->mS) < SAFE_DISTANCE_TO_MAINTAIN)) {
      too_close = true;
      int farthestLeftLane = FARTHEST_LEFT_LANE;
      int farthestRightLane = FARTHEST_RIGHT_LANE;

      if (m_currentLane == FARTHEST_LEFT_LANE) {
        farthestLeftLane = m_currentLane;
        // farthestRightLane = m_currentLane + 1;
        farthestRightLane = FARTHEST_RIGHT_LANE;
      } else if (m_currentLane == FARTHEST_RIGHT_LANE) {
        // farthestLeftLane = m_currentLane - 1;
        farthestLeftLane = FARTHEST_LEFT_LANE;
        farthestRightLane = m_currentLane;
      } else {
        farthestLeftLane = m_currentLane - 1;
        farthestRightLane = m_currentLane + 1;
      }

      for (int i = farthestLeftLane; i <= farthestRightLane; ++i) {
        double newVel = m_refVel;
        double reduce_or_increase_by = VEL_FACTOR;
        if (i == m_currentLane) {
          // Reduce the velocity by a factor of how close we are to the other vehicle
          double factor = ((check_car_s - m_pEgoVehicle->mS) / SAFE_DISTANCE_TO_MAINTAIN);
          reduce_or_increase_by = 1.0 / factor * VEL_FACTOR;
        }
        // Increase velocity if changing lanes otherwise reduce speed if continuing on the same lane
        newVel = reduceOrIncreaseReferenceVelocity(i == m_currentLane, m_refVel, reduce_or_increase_by);
        ego_vehicle_states.push_back(new EgoVehicleNewState(i, newVel));
      }
    }
  }

  if (!too_close) {
    // If the cars are not too close, then continue on the current path and increase velocity.
    double newVel = reduceOrIncreaseReferenceVelocity(false, m_refVel, VEL_FACTOR);
    ego_vehicle_states.push_back(new EgoVehicleNewState(m_currentLane, newVel));
  }

  return ego_vehicle_states;
}

// Change the reference velocity for any vehicle.
double PathPlanner::reduceOrIncreaseReferenceVelocity(
    const bool too_close,
    const double oldRefVel,
    const double reduce_or_increase_by) {
  double newRefVel = oldRefVel;

  if (too_close) {
    newRefVel -= reduce_or_increase_by;
  } else if (oldRefVel < MAX_SPEED) {
    newRefVel += reduce_or_increase_by;
  }

  newRefVel = changeVelocity(newRefVel);

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

EgoVehicleNewState* PathPlanner::selectRandomLane(
    const vector<EgoVehicleNewState *> &possibleEgoVehicleNewStates) const {
  time_t t;
  srand((unsigned) time(&t));
  int total_size = possibleEgoVehicleNewStates.size();
  int selected_state = rand() % total_size;
  EgoVehicleNewState *pNewState = possibleEgoVehicleNewStates[selected_state];
  return pNewState;
}

void PathPlanner::implementBestNewState(const int new_lane, const double new_vel) {
  m_currentLane = new_lane;
  m_refVel = new_vel;
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

  m_counter += 1;

  int prev_path_size = previous_path_x.size();

  double ego_car_s = prev_path_size > 0 ? end_path_s : car_s;
  double ego_car_d = prev_path_size > 0 ? end_path_d : car_d;

  // Initialization of Ego Vehicle
  initializeEgoVehicle(car_x, car_y, car_speed, ego_car_s, ego_car_d, car_yaw);

  // Initialize the traffic. Create Vehicle objects for the id's not yet seen and then find out the closest
  // vehicles and get them in a vector.
  initializeTraffic(sensor_fusion);

  // Check CLoseness with other cars.
  vector<EgoVehicleNewState*> possibleEgoVehicleNewStates;
  possibleEgoVehicleNewStates = checkClosenessToOtherCarsAndChangeLanes(prev_path_size);

  int best_lane = m_currentLane;
  double best_vel = m_refVel;

  if (possibleEgoVehicleNewStates.size() == 1) {
    // If we have 1 state only possible don't bother calculating costs.
    best_lane = possibleEgoVehicleNewStates[0]->new_lane;
    best_vel = possibleEgoVehicleNewStates[0]->refVel;
  } else {
#ifdef MULTITHREADED
    vector<future<double>> future_results;
#else
    vector<double> future_results;
#endif

    if (possibleEgoVehicleNewStates.size() > 1) {
        for (int i = 0; i < possibleEgoVehicleNewStates.size(); ++i) {
        EgoVehicleNewState *pNewState = possibleEgoVehicleNewStates[i];
#ifdef MULTITHREADED
        future<double> res = async(&PathPlanner::computeCostForTrajectory,
                                   this,
                                   pNewState,
                                   previous_path_x,
                                   previous_path_y);
        future_results.push_back(std::move(res));
        double cost = computeCostForTrajectory(
              this,
              pNewState,
              previous_path_x,
              previous_path_y);
        future_results.push_back(cost);
#else
        double cost = computeCostForTrajectory(
              pNewState,
              previous_path_x,
              previous_path_y);
        future_results.push_back(cost);
#endif
      }
    }

    double min_cost = numeric_limits<double>::max();

    for (int i = 0; i < future_results.size(); ++i) {
#ifdef MULTITHREADED
      double current_cost = future_results[i].get();
#else
      double current_cost = future_results[i];
#endif

      cout << "Lane: " << i << " Cost: " << current_cost;
      if (i == m_currentLane)
        cout << " <--- Current Lane";
      cout << endl;
      // For each trajectory generated find out the cost function.
      if (current_cost < min_cost) {
        best_lane = possibleEgoVehicleNewStates[i]->new_lane;
        best_vel = possibleEgoVehicleNewStates[i]->refVel;
        min_cost = current_cost;
      }
    }
  }

  if (possibleEgoVehicleNewStates.size() > 0) {
    cout << "Total States: " << possibleEgoVehicleNewStates.size() << endl;
    cout << "Choosing Best Lane: " << best_lane << endl;
    cout << "-----------------------------------------------------" << endl;
  }
  // EgoVehicleNewState *pNewState = selectRandomLane(possibleEgoVehicleNewStates);
  implementBestNewState(best_lane, best_vel);

  auto p = createTrajectoryPoints(
        m_pEgoVehicle,
        m_currentLane,
        previous_path_x,
        previous_path_y);
  return p;
}
