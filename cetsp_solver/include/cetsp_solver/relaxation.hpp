#pragma once
#include "data.hpp"
#include "node.hpp"
#include "soc.hpp"
#include <gurobi_c++.h>
#include <algorithm>

namespace cetsp_solver {

using RelaxedSolution = std::pair<std::vector<Point>, double>;
class AnnotatedRelaxedSolution {
  /**
   * @brief This class represents a relaxed solution annotated with the
   * distances to the circles for feasibility checking and deciding
   * for the next circle to be added to the sequence. As these calculations
   * are not cheap, it should be used in a two-step process: first, compute
   * only the trajectory and the objective value, and then, if the node
   * is selected for further processing, compute the distances with this
   * class.
   */
public:
  AnnotatedRelaxedSolution(Instance *instance,
                           const RelaxedSolution &relaxed_solution)
      : instance{instance}, trajectory(relaxed_solution.first) {
    compute_distances();
  }

  double is_covered(int idx, double eps = 0.0001) const {
    return distances[idx] <= eps;
  }

  double is_spanning(int idx, double eps = 0.0001) const {
    return distances[idx] >= -eps;
  }

  bool is_feasible(double eps = 0.0001) const {
    return std::all_of(distances.begin(), distances.end(),
                       [eps](double d) { return d <= eps; });
  }

  /**
   * @brief Get the distance to the circle with index idx.
   * 
   * @param idx 
   * @return double 
   */
  double get_distance(int idx) const { return distances.at(idx); }

// Get the index and the corresponding distance of the circle with the maximum distance
// to the trajectory
  std::pair<int, double> get_max_distance() const {
    auto max_it = std::max_element(distances.begin(), distances.end());
    return {std::distance(distances.begin(), max_it), *max_it};
  }

  Instance *instance;
  std::vector<Point> trajectory;

private:
  std::pair<Point, double> compute_distance(int idx) const {
    // compute distance and closest point of the circle with index idx
    // to the trajectory
    const auto &circle = (*instance)[idx];
    assert(trajectory.size() > 1); // Trajectory must have at least two points
    auto segment = Segment(trajectory[0], trajectory[(1) % trajectory.size()]);
    auto [cp, t] = closest_point(segment, circle.center);
    auto squared_dist = squared_distance(cp, circle.center);
    for (uint64_t j = 1; j < trajectory.size(); j++) {
      segment = Segment(trajectory[j], trajectory[(j + 1) % trajectory.size()]);
      auto [cp_, t_] = closest_point(segment, circle.center);
      auto squared_dist_ = squared_distance(cp_, circle.center);
      if (squared_dist_ < squared_dist) {
        cp = cp_;
        t = t_;
        squared_dist = squared_dist_;
      }
    }
    return {cp, std::sqrt(squared_dist) - circle.radius};
  }

  void compute_distances() {
    // Compute the closest point on the trajectory to each circle
    distances.reserve(instance->size());
    closest_points.reserve(instance->size());
    for (uint64_t i = 0; i < instance->size(); i++) {
      auto [cp, t] = compute_distance(i);
      distances.push_back(t);
      closest_points.push_back(cp);
    }
  }
  std::vector<Point> closest_points;
  std::vector<double> distances;
};


} // namespace cetsp_solver