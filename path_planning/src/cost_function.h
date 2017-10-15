#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <math.h>

double logistic(double x);
double too_many_cars_in_target_lane(const int target_lane,
                                    const std::unordered_map<int, std::vector<Vehicle*>>& m_LaneIdToVehicles)


double compute_cost(const Path* pPath,
                    const int target_lane,
                    const std::unordered_map<int, std::vector<Vehicle*>>& m_LaneIdToVehicles);

#endif // COST_FUNCTION_H
