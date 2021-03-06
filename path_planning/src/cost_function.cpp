#include <math.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include <limits>

#include "vehicle.h"
#include "constants.h"
#include "cost_function.h"
#include "utils.h"
#include "path_planner.h"

using namespace std;

double logistic(double x) {
  return 1.0 / (1.0 + exp(-x));
}

double get_car_speed(Vehicle *pCar) {
  double vx = pCar->vx;
  double vy = pCar->vy;
  double check_speed = sqrt(vx * vx + vy * vy);
  return check_speed;
}

double not_in_middle_lane_cost(const int target_lane) {
  return 0.01 * abs(target_lane - MIDDLE_LANE);
}

double too_many_cars_in_target_lane_cost(const int target_lane,
                                         const unordered_map<int, vector<Vehicle*>>& laneIdToVehicles) {
  // Penalize the lanes based on the number of cars and on the avg speed of the cars.
 if (laneIdToVehicles.find(target_lane) == laneIdToVehicles.end())
   return 0.0;

  vector<Vehicle*> cars_in_target_lane = laneIdToVehicles.at(target_lane);
  int num_cars = cars_in_target_lane.size();

  double avg_v = 0;
  for (auto vehicle : cars_in_target_lane) {
    avg_v += sqrt(vehicle->vx * vehicle->vx + vehicle->vy * vehicle->vy);
  }

  avg_v = avg_v / num_cars;
  return logistic(avg_v);
}

// If we are trying to transition more than 1 lane at a time then make sure that no vehicles
// in the path actually collide with us. For example, from lane 0 to lane 2 make sure no one
// hits us both on 1 & 2.
double collision_cost(const Path *pPath,
                      const EgoVehicle *pEgoVehicle,
                      const int current_lane,
                      const int target_lane,
                      const unordered_map<int, vector<Vehicle*>>& laneIdToVehicles,
                      const vector<double> &maps_x,
                      const vector<double> &maps_y) {
  int lower = current_lane < target_lane ? current_lane : target_lane;
  int higher = current_lane > target_lane ? current_lane : target_lane;

  double cost = 0.0;

  for (int lane = lower; lane <= higher; ++lane) {
    if (lane == current_lane)
      continue;

    if (laneIdToVehicles.find(lane) == laneIdToVehicles.end()) {
      cout << "No cars in lane: " << lane << endl;
      continue;
    }

    cout << "Current Lane: " << current_lane << " Target Lane: " << target_lane << endl;
    cout << "Cars in Lane " << lane << " for Collision: " << laneIdToVehicles.at(lane).size() << endl;
    vector<Vehicle*> const cars_in_lane = laneIdToVehicles.at(lane);

    int best_t = numeric_limits<int>::max();

    double best_distance = XY_SAFE_DISTANCE;
    for (Vehicle* pCar : cars_in_lane) {
      // Find out if the car can hit of any of the ego car's trajectory
      // First figure out the speed of the other car
      double car_speed_s = get_car_speed(pCar);
      int total_points = pPath->x_vals.size();
      for (int t = 0; t < total_points; ++t) {
        double tx = pPath->x_vals[t];
        double ty = pPath->y_vals[t];

        // Assuming constant velocity of the other car find out where that car should be
        // at time t.
        double other_car_x = pCar->x + pCar->vx * t;
        double other_car_y = pCar->y + pCar->vy * t;
        double distance = sqrt((tx - other_car_x) * (tx - other_car_x) +
                               (ty - other_car_y) * (ty - other_car_y));


        if (distance < best_distance) {
          // cout << "Distance: " << distance << " Lane: " << lane << endl;
          // We have a collision here. Penalize the cost.
          cost = (1 - (distance / XY_SAFE_DISTANCE)) * 100; // Basically the percentage.
          best_t = t;
          best_distance = distance;
        }
      }
    }
  }

  return cost;
}

double collision_cost_frenet(const Path *pPath,
                      const EgoVehicle *pEgoVehicle,
                      const int current_lane,
                      const int target_lane,
                      const unordered_map<int, vector<Vehicle*>>& laneIdToVehicles,
                      const vector<double> &maps_x,
                      const vector<double> &maps_y) {
  int lower = current_lane < target_lane ? current_lane : target_lane;
  int higher = current_lane > target_lane ? current_lane : target_lane;
  if (lower == higher)
    higher += 1;

  double cost = 0.0;

  // cout << "Lower Lane: " << lower << " Higher Lane: " << higher << endl;
  for (int lane = lower; lane < higher; ++lane) {
    if (laneIdToVehicles.find(lane) == laneIdToVehicles.end())
      continue;
    // cout << "Cars in Lane for Collision: " << laneIdToVehicles.at(lane).size() << endl;
    vector<Vehicle*> const cars_in_lane = laneIdToVehicles.at(lane);
    for (Vehicle* pCar : cars_in_lane) {
      // Find out if the car can hit of any of the ego car's trajectory
      // First figure out the speed of the other car
      double car_speed_s = get_car_speed(pCar);
      int total_points = pPath->x_vals.size();
      double angle = pEgoVehicle->mYaw;

      int best_t = numeric_limits<int>::max();
      double prev_s = pEgoVehicle->mS;
      double prev_d = pEgoVehicle->mD;

      for (int t = 0; t < total_points; ++t) {
        double tx = pPath->x_vals[t];
        double ty = pPath->y_vals[t];

        vector<double> ego_future_sd = getFrenet(tx, ty, angle, maps_x, maps_y);
        // compute if there is any chance of collision with the other car
        double ego_s = ego_future_sd[0];
        double ego_d = ego_future_sd[1];

        // Assuming constant velocity of the other car find out where that car should be
        // at time t.
        double other_car_s = pCar->s + car_speed_s * t * 0.02;

        double distance = abs(ego_s - other_car_s);
        cout << "Distance: " << distance << " Car Id: " << pCar->id << " Car S: " << other_car_s << " Ego Car S: " << ego_s << endl;
        if (distance < 2 * XY_SAFE_DISTANCE_FRENET && t < best_t) {
          // We have a collision here. Penalize the cost.
          cost = ((distance * 10.0) / (t + 1));
          best_t = t;
        }

        angle = atan2((ego_s - prev_s), (ego_d - prev_d));
      }
    }
  }

//  if (cost != 0.0)
//    cout << "Car will collide changing lanes from " << current_lane << " to " << target_lane << endl;
  return cost;
}

double less_than_optimum_velocity(double current_velocity) {
  double cost = 10 * (MAX_SPEED - current_velocity);
  return cost;
}

double compute_cost(const Path *pPath,
                    const EgoVehicle *pEgoVehicle,
                    const int current_lane,
                    const int target_lane,
                    const double target_speed,
                    const unordered_map<int, vector<Vehicle *>> &laneIdToVehicles,
                    const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double cost = 0;
  double cost_not_in_middle_lane = not_in_middle_lane_cost(target_lane);
  // cout << "=======================================================" << endl;
  // cout << "Lane " << target_lane << " Current Lane: " << current_lane << endl;
  // cout << "Not in middle lane for lane: " << target_lane << " --> " << cost_not_in_middle_lane << endl;
  double cost_too_many_cars_in_target_lane = too_many_cars_in_target_lane_cost(target_lane, laneIdToVehicles);

  //cout << "Too many cars in target lane: " << target_lane << " --> " << cost_too_many_cars_in_target_lane << endl;
  double cost_collision = collision_cost(
                                         pPath,
                                         pEgoVehicle,
                                         current_lane,
                                         target_lane,
                                         laneIdToVehicles,
                                         maps_x,
                                         maps_y);
  // cout << "Collision cost for lane " << target_lane << ": " << cost_collision << endl;

  double cost_less_than_optimum_velocity = less_than_optimum_velocity(target_speed);

  // cout << "Less Than optimum velocity for lane " << target_lane << ": " << cost_less_than_optimum_velocity << endl;
  // cout << "=======================================================" << endl;
  cost = cost_not_in_middle_lane + cost_too_many_cars_in_target_lane + cost_collision + cost_less_than_optimum_velocity;
  return cost;
}
