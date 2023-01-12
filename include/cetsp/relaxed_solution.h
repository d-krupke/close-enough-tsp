//
// Created by Dominik Krupke on 12.01.23.
//

#ifndef CETSP_RELAXED_SOLUTION_H
#define CETSP_RELAXED_SOLUTION_H
#include "cetsp/common.h"
#include "cetsp/soc.h"
#include <vector>

namespace cetsp {

class PartialSequenceSolution {
  /**
   * This class simplifies the handling of the relaxed solution that is based
   * on a sequence of circles, that allow to compute the optimal tour respecting
   * this order using a second order cone program.
   */
public:
  PartialSequenceSolution(const Instance *instance, std::vector<int> sequence_,
                          double feasibility_tol = 0.01)
      : instance{instance}, sequence{std::move(sequence_)},
        FEASIBILITY_TOL{feasibility_tol} {
    if (sequence.empty() && !instance->is_path()) {
      throw std::invalid_argument(
          "Cannot compute tour trajectory from empty sequence.");
    }
    assert(std::all_of(sequence.begin(), sequence.end(), [&instance](auto i) {
      return i < static_cast<int>(instance->size());
    }));
    if (instance->is_tour()) {
      compute_tour_trajectory();
    } else {
      compute_path_trajectory();
    }
  }

  /**
   * Returns true if the i-th circle in the sequence is spanning. This
   * information is useful to simplify the solution.
   * @param i The index of the circle within the sequence. Not the index of  the
   * circle in the solution.
   * @return True if it spans the trajectory.
   */
  bool is_sequence_index_spanning(int i) const { return spanning[i]; }

  const Point &trajectory_begin() const { return trajectory.points.front(); }

  const Point &trajectory_end() const { return trajectory.points.back(); }

  /**
   * Returns the point that covers the i-th circle in the sequence. These
   * are the potential turning points in the trajectory.
   * @param i
   * @return
   */
  const Point &get_sequence_hitting_point(int i) const {
    if (instance->is_tour()) {
      return trajectory.points[i];
    } else {
      return trajectory.points[i + 1];
    }
  }

  const Trajectory &get_trajectory() const { return trajectory; }

  const std::vector<int> &get_sequence() const { return sequence; }

  double obj() const { return trajectory.length(); }

  bool is_feasible() {
    if (!_feasible) {
      _feasible = trajectory.covers(instance->begin(), instance->end(),
                                    FEASIBILITY_TOL);
    }
    return *_feasible;
  }

  /**
   * Simplify the sequence and the solution by removing implicitly covered
   * parts.
   */
  void simplify() {
    if(simplified) {
      return;
    }
    std::vector<Point> points;
    std::vector<int> simplified_sequence;
    std::vector<bool> is_spanning;
    if (instance->is_path()) {
      // Trajectory of a path has a fixed beginning, not represented in the
      // sequence
      points.push_back(trajectory_begin());
    }
    // add all spanning circles and their hitting points
    for (int i = 0; i < sequence.size(); ++i) {
      if (is_sequence_index_spanning(i)) {
        points.push_back(get_sequence_hitting_point(i));
        simplified_sequence.push_back(sequence[i]);
        is_spanning.push_back(true);
      }
    }
    // Close the trajectory
    if (instance->is_path()) {
      // Trajectory of a path has a fixed ending, not represented in the
      // sequence
      points.push_back(trajectory_end());
    } else {
      // close the tour by going back to  the beginning
      points.push_back(points.front());
    }
    // update the trajectory and sequence. Feasibility etc. doesn't change.
    trajectory = Trajectory(std::move(points));
    sequence = std::move(simplified_sequence);
    spanning = std::move(is_spanning);
    simplified = true;
  }

private:
  void compute_tour_trajectory() {
    // compute the optimal tour trajectory through the sequence
    std::vector<Circle> circles;
    circles.reserve(sequence.size());
    for (auto i : sequence) {
      assert(i < static_cast<int>(instance->size()));
      circles.push_back((*instance).at(i));
    }
    assert(circles.size() == sequence.size());
    auto soc = compute_tour_with_spanning_information(circles, false);
    trajectory = std::move(soc.first);
    spanning = std::move(soc.second);
  }

  void compute_path_trajectory() {
    // compute the optimal path trajectory through the sequence
    std::vector<Circle> circles;
    circles.reserve(sequence.size() + 2);
    circles.emplace_back(instance->path->first, 0);
    for (auto i : sequence) {
      circles.push_back((*instance).at(i));
    }
    circles.emplace_back(instance->path->second, 0);
    assert(circles.size() == sequence.size() + 2);
    auto soc = compute_tour_with_spanning_information(circles, true);
    trajectory = std::move(soc.first);
    spanning.reserve(sequence.size());
    for (int i = 1; i < soc.second.size() - 1; ++i) {
      spanning[i - 1] = soc.second[i];
    }
  }

  const Instance *instance;
  std::vector<int> sequence;
  Trajectory trajectory;
  std::vector<bool> spanning;
  std::optional<bool> _feasible;
  bool simplified = false;
  double FEASIBILITY_TOL;
};

TEST_CASE("PartialSequentialSolution") {
  Instance instance(
      {{{0, 0}, 0.01}, {{3, 0}, 0.01}, {{3, 3}, 0.01}, {{0, 3}, 0.01}});
  PartialSequenceSolution pss(&instance, {0, 1, 2, 3});
  CHECK(pss.obj() == doctest::Approx(11.9434));
  pss.simplify();
  CHECK(pss.obj() == doctest::Approx(11.9434));
  CHECK(pss.is_feasible());
}
} // namespace cetsp

#endif // CETSP_RELAXED_SOLUTION_H
