#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <math.h>
#include <vector>
#include <unordered_map>

class Path;

double logistic(double x);

/*                    Cost Functions               */
double not_in_middle_lane_cost(const int target_lane);

double too_many_cars_in_target_lane_cost(const int target_lane,
                                         const std::unordered_map<int, std::vector<Vehicle*>>& laneIdToVehicles);

double collision_cost(const Path *pPath,
                      const EgoVehicle *pEgoVehicle,
                      const int current_lane,
                      const int target_lane,
                      const std::unordered_map<int, std::vector<Vehicle*>>& laneIdToVehicles,
                      const std::vector<double> &maps_x,
                      const std::vector<double> &maps_y);

double compute_cost(const Path* pPath,
                    const EgoVehicle *pEgoVehicle,
                    const int current_lane,
                    const int target_lane,
                    const double target_speed,
                    const std::unordered_map<int, std::vector<Vehicle*>>& laneIdToVehicles,
                    const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

#endif // COST_FUNCTION_H
