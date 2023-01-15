/**
 * An important observation for  the close  enought TSP is that if the order
 * of the circles to be visited is given, we can compute the optimal trajectory
 * via a second order cone program (poly-time). This file provides this
 * functionality.
 *
 * It is probably easier to use via RelaxedSequenceSolution.
 *
 * Dominik Krupke, January 2023, Tel Aviv
 */
#ifndef CETSP_SOC_H
#define CETSP_SOC_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
#include <vector>
namespace cetsp {

/**
 * Computes the shortest tour through the sequence of circles. Will also give
 * you information which circles are tour defining, i.e., their hitting point
 * induces a turn and skipping this circle would shorten the trajectory.
 * Circles that are not tour defining can theoretically be remove from the
 * sequence without changing it.
 * @param circle_sequence A sequence of circles.
 * @param path Defines if we want a tour or a path. The  tour will return to the
 * hitting point of the first circle, closing the trajectory. Note that the path
 * is not just a  tour  with one segment  missing, but can look completely
 * different.
 * @return The trajectory and  a list of  boolean  of  the same length  as the
 * input sequence, stating if the circle is tour defining.
 */
std::pair<Trajectory, std::vector<bool>>
compute_trajectory_with_information(const std::vector<Circle> &circle_sequence,
                                    bool path);

/**
 * Like `compute_trajectory_with_information`  but throwing away the
 * additional information, only returning the trajectory.
 */
Trajectory compute_tour(const std::vector<Circle> &circle_sequence,
                        bool path = false);

TEST_CASE("Simple SOCP test") {
  // define  the circle sequence  we want to have the trajectory for
  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}};
  // compute the tour trajectory
  auto traj = compute_tour(seq, false);
  CHECK(traj.length() == doctest::Approx(2));
  // compute the path trajectory
  traj = compute_tour(seq, true);
  CHECK(traj.length() == doctest::Approx(1));
}
} // namespace cetsp
#endif
