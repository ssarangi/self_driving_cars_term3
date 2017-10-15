#include <math.h>
#include <vector>

#include "constants.h"
#include "cost_function.h"

using namespace std;

double logistic(double x) {
  return 1.0 / (1.0 + exp(-x));
}

double not_in_middle_lane(const int target_lane) {
  return logistic(abs(target_lane - MIDDLE_LANE));
}

double too_many_cars_in_target_lane(const int target_lane,
                                    const std::unordered_map<int, std::vector<Vehicle*>>& m_LaneIdToVehicles) {
  // Penalize the lanes based on the number of cars and on the avg speed of the cars.
  vector<Vehicle*> cars_in_target_lane = m_LaneIdToVehicles[target_lane];
  int num_cars = cars_in_target_lane.size();

  double avg_v = 0;
  for (auto vehicle : cars_in_target_lane) {
    avg_v += sqrt(vehicle->vx * vehicle->vx + vehicle->vy * vehicle->vy);
  }

  avg_v = avg_v / num_cars;
  return logistic(avg_v);
}

double compute_cost(const Path *pPath,
                    const int target_lane,
                    const std::unordered_map<int, std::vector<Vehicle *> > &m_LaneIdToVehicles) {

}
