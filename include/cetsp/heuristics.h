//
// Created by Dominik Krupke on 11.12.22.
//

#ifndef CETSP_HEURISTICS_H
#define CETSP_HEURISTICS_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
namespace cetsp {
/**
 * Compute a heuristic solution using a procedure based on 2-Opt.
 * For this, only the circle's centers are considered, which
 * can lead to quite suboptimal solutions in some cases.
 */
auto compute_tour_by_2opt(Instance &instance) -> Trajectory;

TEST_CASE("2Opt") {
  const std::vector<Circle> seq = {
      {{0, 0}, 0}, {{1, 1}, 0}, {{1, 0}, 0}, {{0, 1}, 0}};
  auto instance = Instance(seq);
  auto traj = compute_tour_by_2opt(instance);
  CHECK(traj.length() == doctest::Approx(4));

  const std::vector<Circle> seq2 = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{5, 2}, 1}, {{3, 3}, 1}, {{0, 4}, 1}};
  auto instance2 = Instance(seq2);
  traj = compute_tour_by_2opt(instance2);
  CHECK(traj.length() >= 1);
}
} // namespace cetsp
#endif // CETSP_HEURISTICS_H
