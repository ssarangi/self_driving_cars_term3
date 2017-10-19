#include <math.h>
#include <vector>
#include <assert.h>

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
  double check_car_s = pCar->s;
  double check_speed = sqrt(vx * vx + vy * vy);

  check_car_s += 0.02 * check_speed;
  return check_car_s;
}

double not_in_middle_lane_cost(const int target_lane) {
  return logistic(abs(target_lane - MIDDLE_LANE));
}

double too_many_cars_in_target_lane_cost(const int target_lane,
                                         const unordered_map<int, vector<Vehicle*>>& laneIdToVehicles) {
  // Penalize the lanes based on the number of cars and on the avg speed of the cars.
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
  assert(current_lane != target_lane);
  int lower = min(current_lane, target_lane);
  int higher = max(current_lane, target_lane);
  double cost = 0.0;

  for (int lane = lower; lane <= higher; ++lane) {
    vector<Vehicle*> const cars_in_lane = laneIdToVehicles.at(lane);
    for (Vehicle* pCar : cars_in_lane) {
      // Find out if the car can hit of any of the ego car's trajectory
      // First figure out the speed of the other car
      double car_speed_s = get_car_speed(pCar);
      int total_points = pPath->x_vals.size();
      double angle = pEgoVehicle->mYaw;
      for (int t = 0; t < total_points; ++t) {
        double tx = pPath->x_vals[t];
        double ty = pPath->y_vals[t];

        vector<double> ego_future_sd = getFrenet(tx, ty, angle, maps_x, maps_y);
        // compute if there is any chance of collision with the other car
        double ego_s = ego_future_sd[0];
        double ego_d = ego_future_sd[1];

        // Assuming constant velocity of the other car find out where that car should be
        // at time t.
        double other_car_s = pCar->s + car_speed_s * t;

        double distance = sqrt((ego_s - other_car_s) * (ego_s - pCar->s) + (ego_d - pCar->d) * (ego_d - pCar->d));
        if (distance < 2 * CAR_RADIUS) {
          // We have a collision here. Penalize the cost.
          cost += logistic(distance * 10.0);
        }

        if (t < total_points) {
          angle = deg2rad(atan(pPath->x_vals[t+1] - tx / pPath->y_vals[t+1] - ty));
        }
      }
    }
  }

  return cost;
}

double compute_cost(const Path *pPath,
                    const EgoVehicle *pEgoVehicle,
                    const int current_lane,
                    const int target_lane,
                    const unordered_map<int, vector<Vehicle *>> &laneIdToVehicles,
                    const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double cost = 0;
  cost += not_in_middle_lane_cost(target_lane);
  cost += too_many_cars_in_target_lane_cost(target_lane, laneIdToVehicles);
  cost += collision_cost(pPath, pEgoVehicle, current_lane, target_lane, laneIdToVehicles, maps_x, maps_y);
  return cost;
}
