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
  return logistic(abs(target_lane - MIDDLE_LANE) * abs(target_lane - MIDDLE_LANE));
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
  int lower = min(current_lane, target_lane);
  int higher = max(current_lane, target_lane);
  double cost = 0.0;

  for (int lane = lower; lane < higher; ++lane) {
    if (current_lane == lane)
      continue;

    if (laneIdToVehicles.find(lane) == laneIdToVehicles.end())
      continue;

    vector<Vehicle*> const cars_in_lane = laneIdToVehicles.at(lane);

    int best_t = numeric_limits<int>::max();

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
        double distance = sqrt((tx - pCar->vx * t) * (tx - pCar->vx * t) +
                               (ty - pCar->vy * t) * (ty - pCar->vy * t));

        if (distance < 2 * 1000.0 && t < best_t) {
          // We have a collision here. Penalize the cost.
          // cost = exp(-(t * t));
          cost = 10;
          best_t = t;
        }
      }
    }
  }

  return cost;
}

//double collision_cost(const Path *pPath,
//                      const EgoVehicle *pEgoVehicle,
//                      const int current_lane,
//                      const int target_lane,
//                      const unordered_map<int, vector<Vehicle*>>& laneIdToVehicles,
//                      const vector<double> &maps_x,
//                      const vector<double> &maps_y) {
//  int lower = min(current_lane + 1, target_lane);
//  int higher = max(current_lane + 1, target_lane);
//  double cost = 0.0;

//  for (int lane = lower; lane < higher; ++lane) {
//    vector<Vehicle*> const cars_in_lane = laneIdToVehicles.at(lane);
//    for (Vehicle* pCar : cars_in_lane) {
//      // Find out if the car can hit of any of the ego car's trajectory
//      // First figure out the speed of the other car
//      double car_speed_s = get_car_speed(pCar);
//      int total_points = pPath->x_vals.size();
//      double angle = pEgoVehicle->mYaw;
//      for (int t = 0; t < total_points; ++t) {
//        double tx = pPath->x_vals[t];
//        double ty = pPath->y_vals[t];

//        vector<double> ego_future_sd = getFrenet(tx, ty, angle, maps_x, maps_y);
//        // compute if there is any chance of collision with the other car
//        double ego_s = ego_future_sd[0];
//        double ego_d = ego_future_sd[1];

//        // Assuming constant velocity of the other car find out where that car should be
//        // at time t.
//        double other_car_s = pCar->s + car_speed_s * t * 0.02;

//        double distance = sqrt((ego_s - other_car_s) * (ego_s - other_car_s) + (ego_d - pCar->d) * (ego_d - pCar->d));
//        //cout << "Distance: " << distance << endl;
//        if (distance < 2 * 50.0) {
//          // We have a collision here. Penalize the cost.
//          cost += logistic(distance);
//        }
//      }
//    }
//  }

//  return cost;
//}

double less_than_optimum_velocity(double current_velocity) {
  double cost = logistic(MAX_SPEED - current_velocity);
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
  cost_not_in_middle_lane = 0.0;
  cout << "Not in middle lane for lane: " << target_lane << " --> " << cost_not_in_middle_lane << endl;
  double cost_too_many_cars_in_target_lane = too_many_cars_in_target_lane_cost(target_lane, laneIdToVehicles);
  cout << "Too many cars in target lane: " << target_lane << " --> " << cost_too_many_cars_in_target_lane << endl;
  double cost_collision = collision_cost(pPath,
                                         pEgoVehicle,
                                         current_lane,
                                         target_lane,
                                         laneIdToVehicles,
                                         maps_x,
                                         maps_y);
  cout << "Collision cost for lane " << target_lane << ": " << cost_collision << endl;

  double cost_less_than_optimum_velocity = less_than_optimum_velocity(target_speed);

  cost = cost_not_in_middle_lane + cost_too_many_cars_in_target_lane + cost_collision + cost_less_than_optimum_velocity;
  return cost;
}
